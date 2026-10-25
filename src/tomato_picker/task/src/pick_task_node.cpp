#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "tomato_picker_interfaces/action/move_arm.hpp"
#include "tomato_picker_interfaces/action/pick_target.hpp"
#include "tomato_picker_interfaces/srv/command_eef.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace tomato_picker_task {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //

namespace {

constexpr std::int32_t kSuccess = 0;                 ///< 成功状态码
constexpr std::int32_t kDependencyUnavailable = 1;   ///< 下游 capability 不可用
constexpr std::int32_t kMotionFailed = 2;            ///< 运动步骤失败
constexpr std::int32_t kEefFailed = 3;               ///< EEF 步骤失败
constexpr std::int32_t kCanceled = 4;                ///< 任务被取消

// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

/**
 * @brief 沿目标位姿局部 Z 轴生成预采摘、接近或退出位姿
 * @param target 原目标位姿
 * @param delta_z 局部 Z 方向偏移 m
 * @return 偏移后的目标位姿
 */
geometry_msgs::msg::PoseStamped offset_local_z(const geometry_msgs::msg::PoseStamped& target, double delta_z) {
    auto result = target;
    const auto& q = target.pose.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if(norm <= 1.0e-9) {
        result.pose.position.z += delta_z;
        return result;
    }

    const double x = q.x / norm;
    const double y = q.y / norm;
    const double z = q.z / norm;
    const double w = q.w / norm;
    result.pose.position.x += 2.0 * (x * z + w * y) * delta_z;
    result.pose.position.y += 2.0 * (y * z - w * x) * delta_z;
    result.pose.position.z += (1.0 - 2.0 * (x * x + y * y)) * delta_z;
    return result;
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 通过 MoveArm Action 与 CommandEef Service 编排单目标采摘流程
 */
class PickTaskNode final : public rclcpp::Node {
public:
    using PickTarget = tomato_picker_interfaces::action::PickTarget;
    using MoveArm = tomato_picker_interfaces::action::MoveArm;
    using CommandEef = tomato_picker_interfaces::srv::CommandEef;
    using GoalHandlePickTarget = rclcpp_action::ServerGoalHandle<PickTarget>;
    using GoalHandleMoveArm = rclcpp_action::ClientGoalHandle<MoveArm>;

    /**
     * @brief 构造采摘任务节点并连接下游 capability
     */
    PickTaskNode()
        : Node("pick_task_node") {
        action_name_ = declare_parameter<std::string>("action_name", "/tomato_picker/task/pick");
        motion_action_name_ = declare_parameter<std::string>("motion_action", "/tomato_picker/motion/move_arm");
        eef_service_name_ = declare_parameter<std::string>("eef_service", "/tomato_picker/eef/command");
        motion_timeout_sec_ = declare_parameter<double>("motion_timeout_sec", 30.0);
        service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 3.0);
        default_pre_pick_distance_ = declare_parameter<double>("default_pre_pick_distance", 0.15);
        default_approach_distance_ = declare_parameter<double>("default_approach_distance", 0.08);
        default_retreat_distance_ = declare_parameter<double>("default_retreat_distance", 0.10);
        motion_velocity_scale_ = declare_parameter<double>("motion_velocity_scale", 0.15);
        motion_acceleration_scale_ = declare_parameter<double>("motion_acceleration_scale", 0.15);

        motion_client_ = rclcpp_action::create_client<MoveArm>(this, motion_action_name_);
        eef_client_ = create_client<CommandEef>(eef_service_name_);
        action_server_ = rclcpp_action::create_server<PickTarget>(
            this,
            action_name_,
            std::bind(&PickTaskNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PickTaskNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&PickTaskNode::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "PickTaskNode started; action=%s", action_name_.c_str());
    }

    /**
     * @brief 取消当前子运动并回收采摘执行线程
     */
    ~PickTaskNode() override {
        shutting_down_.store(true);
        cancel_requested_.store(true);
        cancel_active_motion_goal();
        if(execution_thread_.joinable()) execution_thread_.join();
    }

private:
    /**
     * @brief 接受单个采摘任务并拒绝并发执行
     * @param uuid Action goal UUID
     * @param goal PickTarget goal
     * @return 接受或拒绝策略
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const PickTarget::Goal> goal) {
        static_cast<void>(uuid);
        if(shutting_down_.load() || motion_pending()) return rclcpp_action::GoalResponse::REJECT;
        if(goal->target_pose.header.frame_id.empty()) return rclcpp_action::GoalResponse::REJECT;
        if(goal->use_pre_pick_pose && goal->pre_pick_pose.header.frame_id.empty()) return rclcpp_action::GoalResponse::REJECT;
        if(goal->use_approach_pose && goal->approach_pose.header.frame_id.empty()) return rclcpp_action::GoalResponse::REJECT;
        if(goal->use_retreat_pose && goal->retreat_pose.header.frame_id.empty()) return rclcpp_action::GoalResponse::REJECT;
        if(goal->use_place_pose && goal->place_pose.header.frame_id.empty()) return rclcpp_action::GoalResponse::REJECT;

        bool expected = false;
        if(!executing_.compare_exchange_strong(expected, true)) return rclcpp_action::GoalResponse::REJECT;
        cancel_requested_.store(false);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * @brief 接受采摘任务取消请求
     * @param goal_handle 当前采摘 goal handle
     * @return 取消策略
     */
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePickTarget> goal_handle) {
        static_cast<void>(goal_handle);
        cancel_requested_.store(true);
        cancel_active_motion_goal();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * @brief 启动独立线程执行采摘阶段机
     * @param goal_handle 已接受采摘 goal handle
     */
    void handle_accepted(const std::shared_ptr<GoalHandlePickTarget> goal_handle) {
        if(execution_thread_.joinable()) execution_thread_.join();
        execution_thread_ = std::thread([this, goal_handle]() {
            try { execute(goal_handle); }
            catch(const std::exception& error) {
                cancel_active_motion_goal();
                std::shared_ptr<PendingMotion> pending;
                { std::lock_guard<std::mutex> lock(active_motion_goal_mutex_); pending = pending_motion_; }
                if(pending) stop_and_wait(pending);
                auto result = std::make_shared<PickTarget::Result>();
                finish_failed(goal_handle, result, kMotionFailed, error.what(), 0);
            }
        });
    }

    /**
     * @brief 执行完整采摘阶段机
     * @param goal_handle 当前采摘 goal handle
     */
    void execute(const std::shared_ptr<GoalHandlePickTarget> goal_handle) {
        auto result = std::make_shared<PickTarget::Result>();
        result->success = false;
        result->error_code = kDependencyUnavailable;

        if(!motion_client_->wait_for_action_server(std::chrono::seconds(3))) {
            result->message = "MoveArm action unavailable";
            goal_handle->abort(result);
            executing_.store(false);
            return;
        }
        if(goal_handle->get_goal()->use_eef && !eef_client_->wait_for_service(std::chrono::seconds(3))) {
            result->message = "CommandEef service unavailable";
            goal_handle->abort(result);
            executing_.store(false);
            return;
        }

        const auto goal = goal_handle->get_goal();
        const double pre_pick_distance = goal->pre_pick_distance > 0.0 ? goal->pre_pick_distance : default_pre_pick_distance_;
        const double approach_distance = goal->approach_distance > 0.0 ? goal->approach_distance : default_approach_distance_;
        const double retreat_distance = goal->retreat_distance > 0.0 ? goal->retreat_distance : default_retreat_distance_;
        const auto pre_pick_pose = goal->use_pre_pick_pose ? goal->pre_pick_pose : offset_local_z(goal->target_pose, pre_pick_distance);
        const auto approach_pose = goal->use_approach_pose ? goal->approach_pose : offset_local_z(goal->target_pose, approach_distance);
        const auto retreat_pose = goal->use_retreat_pose ? goal->retreat_pose : offset_local_z(goal->target_pose, retreat_distance);

        std::uint32_t completed = 0;
        const std::uint32_t total_steps = 4 + (goal->use_eef ? 1U : 0U) + (goal->use_place_pose ? 1U : 0U) +
            ((goal->use_eef && goal->use_place_pose) ? 1U : 0U) + (goal->go_home_after_finish ? 1U : 0U);

        if(!run_pose_step(goal_handle, pre_pick_pose, PickTarget::Feedback::STAGE_PRE_PICK, "PRE_PICK", completed, total_steps, goal->retry_times)) return finish_failed(goal_handle, result, kMotionFailed, "pre-pick failed", completed);
        ++completed;
        if(!run_pose_step(goal_handle, approach_pose, PickTarget::Feedback::STAGE_APPROACH, "APPROACH", completed, total_steps, goal->retry_times)) return finish_failed(goal_handle, result, kMotionFailed, "approach failed", completed);
        ++completed;
        if(!run_pose_step(goal_handle, goal->target_pose, PickTarget::Feedback::STAGE_PICK, "PICK", completed, total_steps, goal->retry_times)) return finish_failed(goal_handle, result, kMotionFailed, "pick pose failed", completed);
        ++completed;

        if(goal->use_eef) {
            publish_feedback(goal_handle, PickTarget::Feedback::STAGE_EEF_CLOSE, completed, total_steps, "EEF_CLOSE");
            if(!run_eef(CommandEef::Request::CLOSE)) return finish_failed(goal_handle, result, kEefFailed, "eef close failed", completed);
            ++completed;
        }

        if(!run_pose_step(goal_handle, retreat_pose, PickTarget::Feedback::STAGE_RETREAT, "RETREAT", completed, total_steps, goal->retry_times)) return finish_failed(goal_handle, result, kMotionFailed, "retreat failed", completed);
        ++completed;

        if(goal->use_place_pose) {
            if(!run_pose_step(goal_handle, goal->place_pose, PickTarget::Feedback::STAGE_PLACE, "PLACE", completed, total_steps, goal->retry_times)) return finish_failed(goal_handle, result, kMotionFailed, "place move failed", completed);
            ++completed;
            if(goal->use_eef) {
                publish_feedback(goal_handle, PickTarget::Feedback::STAGE_EEF_OPEN, completed, total_steps, "EEF_OPEN");
                if(!run_eef(CommandEef::Request::OPEN)) return finish_failed(goal_handle, result, kEefFailed, "eef open failed", completed);
                ++completed;
            }
        }

        if(goal->go_home_after_finish) {
            publish_feedback(goal_handle, PickTarget::Feedback::STAGE_HOME, completed, total_steps, "HOME");
            if(!run_home()) return finish_failed(goal_handle, result, kMotionFailed, "home failed", completed);
            ++completed;
        }

        if(cancel_requested_.load() || goal_handle->is_canceling()) {
            finish_failed(goal_handle, result, kCanceled, "pick task canceled", completed);
            return;
        }

        result->success = true;
        result->error_code = kSuccess;
        result->message = "pick task completed";
        result->completed_steps = completed;
        result->canceled = false;
        result->final_pose = last_pose_;
        goal_handle->succeed(result);
        executing_.store(false);
    }

    /**
     * @brief 执行一个位姿运动步骤
     * @param goal_handle PickTarget goal handle
     * @param target 目标位姿
     * @param stage_id 阶段编号
     * @param stage 阶段名称
     * @param completed 已完成步骤数
     * @param total_steps 总步骤数
     * @param retry_times 失败后的最大重试次数
     * @return 执行成功返回 true
     */
    bool run_pose_step(const std::shared_ptr<GoalHandlePickTarget> goal_handle, const geometry_msgs::msg::PoseStamped& target,
        std::uint8_t stage_id, const std::string& stage, std::uint32_t completed, std::uint32_t total_steps, std::uint8_t retry_times) {
        MoveArm::Goal motion_goal;
        motion_goal.command_type = MoveArm::Goal::POSE;
        motion_goal.target_pose = target;
        motion_goal.velocity_scale = motion_velocity_scale_;
        motion_goal.acceleration_scale = motion_acceleration_scale_;
        motion_goal.execute = true;

        for(std::uint16_t attempt = 0; attempt <= retry_times; ++attempt) {
            if(cancel_requested_.load() || motion_pending()) return false;
            publish_feedback(goal_handle, stage_id, completed, total_steps, attempt == 0 ? stage : stage + "_RETRY");
            if(run_motion(motion_goal)) return true;
        }
        return false;
    }

    /**
     * @brief 请求机械臂回到 named home target
     * @return 执行成功返回 true
     */
    bool run_home() {
        MoveArm::Goal motion_goal;
        motion_goal.command_type = MoveArm::Goal::HOME;
        motion_goal.velocity_scale = motion_velocity_scale_;
        motion_goal.acceleration_scale = motion_acceleration_scale_;
        motion_goal.execute = true;
        return run_motion(motion_goal);
    }

    /**
     * @brief 同步等待一个 MoveArm 子 Action 完成
     * @param motion_goal MoveArm goal
     * @return 子 Action 成功返回 true
     */
    struct PendingMotion {
        std::mutex mutex;
        GoalHandleMoveArm::SharedPtr handle;
        std::atomic_bool abandoned{false};
        std::atomic_bool terminal{false};
    };

    bool motion_pending() {
        std::lock_guard<std::mutex> lock(active_motion_goal_mutex_);
        return pending_motion_ && !pending_motion_->terminal.load();
    }

    bool stop_and_wait(const std::shared_ptr<PendingMotion>& pending) {
        pending->abandoned.store(true);
        GoalHandleMoveArm::SharedPtr handle;
        { std::lock_guard<std::mutex> lock(pending->mutex); handle = pending->handle; }
        if(handle && !pending->terminal.load()) {
            try { motion_client_->async_cancel_goal(handle); }
            catch(const std::exception& error) {
                RCLCPP_WARN(get_logger(), "Motion cancellation is unconfirmed: %s", error.what());
            }
        }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while(!pending->terminal.load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        return pending->terminal.load();
    }

    bool run_motion(const MoveArm::Goal& motion_goal) {
        if(cancel_requested_.load() || shutting_down_.load()) return false;

        if(motion_pending()) return false;
        auto pending = std::make_shared<PendingMotion>();
        {
            std::lock_guard<std::mutex> lock(active_motion_goal_mutex_);
            pending_motion_ = pending;
        }
        rclcpp_action::Client<MoveArm>::SendGoalOptions options;
        std::weak_ptr<rclcpp_action::Client<MoveArm>> client = motion_client_;
        options.goal_response_callback = [pending, client](GoalHandleMoveArm::SharedPtr handle) {
            { std::lock_guard<std::mutex> lock(pending->mutex); pending->handle = handle; }
            if(!handle) { pending->terminal.store(true); return; }
            if(pending->abandoned.load()) {
                if(auto locked = client.lock()) {
                    try { locked->async_cancel_goal(handle); }
                    catch(const std::exception& error) {
                        RCLCPP_ERROR(rclcpp::get_logger("pick_task_node"), "Late motion cancellation is unconfirmed: %s", error.what());
                    }
                }
            }
        };
        options.result_callback = [pending](const GoalHandleMoveArm::WrappedResult&) {
            pending->terminal.store(true);
        };
        auto send_future = motion_client_->async_send_goal(motion_goal, options);
        const auto send_deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(motion_timeout_sec_);
        while(send_future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
            if(cancel_requested_.load() || shutting_down_.load() || std::chrono::steady_clock::now() >= send_deadline) {
                stop_and_wait(pending);
                return false;
            }
        }
        auto active_goal = send_future.get();
        if(!active_goal) return false;
        if(cancel_requested_.load() || shutting_down_.load()) {
            stop_and_wait(pending);
            return false;
        }
        auto result_future = motion_client_->async_get_result(active_goal);
        const auto result_deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(motion_timeout_sec_);
        while(result_future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
            if(cancel_requested_.load() || shutting_down_.load() || std::chrono::steady_clock::now() >= result_deadline) {
                stop_and_wait(pending);
                return false;
            }
        }
        pending->terminal.store(true);
        const auto wrapped_result = result_future.get();
        if(wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped_result.result || !wrapped_result.result->success) return false;
        last_pose_ = wrapped_result.result->final_pose;
        return true;
    }

    /**
     * @brief 同步等待一个 EEF Service 命令完成
     * @param command CommandEef 命令枚举
     * @return 服务成功返回 true
     */
    bool run_eef(std::uint8_t command) {
        if(cancel_requested_.load() || shutting_down_.load()) return false;
        auto request = std::make_shared<CommandEef::Request>();
        request->command = command;
        request->value = 0.0;
        auto future = eef_client_->async_send_request(request);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(service_timeout_sec_);
        while(future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
            if(cancel_requested_.load() || shutting_down_.load() || std::chrono::steady_clock::now() >= deadline) return false;
        }
        return future.get()->success;
    }

    /**
     * @brief 请求取消当前 MoveArm 子 goal
     */
    void cancel_active_motion_goal() {
        std::shared_ptr<PendingMotion> pending;
        { std::lock_guard<std::mutex> lock(active_motion_goal_mutex_); pending = pending_motion_; }
        if(!pending) return;
        pending->abandoned.store(true);
        GoalHandleMoveArm::SharedPtr handle;
        { std::lock_guard<std::mutex> lock(pending->mutex); handle = pending->handle; }
        if(!handle || pending->terminal.load()) return;
        try { motion_client_->async_cancel_goal(handle); }
        catch(const std::exception& error) {
            RCLCPP_WARN(get_logger(), "Motion stop not confirmed: %s", error.what());
        }
    }

    /**
     * @brief 发布采摘阶段反馈
     * @param goal_handle 当前 PickTarget goal handle
     * @param stage 阶段编号
     * @param completed 已完成步骤数
     * @param total_steps 总步骤数
     * @param text 阶段文本
     */
    void publish_feedback(const std::shared_ptr<GoalHandlePickTarget> goal_handle, std::uint8_t stage, std::uint32_t completed,
        std::uint32_t total_steps, const std::string& text) {
        auto feedback = std::make_shared<PickTarget::Feedback>();
        feedback->current_stage = stage;
        feedback->completed_steps = completed;
        feedback->total_steps = total_steps;
        feedback->stage_text = text;
        goal_handle->publish_feedback(feedback);
    }

    /**
     * @brief 统一结束失败或取消的采摘任务
     * @param goal_handle 当前 PickTarget goal handle
     * @param result Action 结果
     * @param error_code 错误码
     * @param message 错误文本
     * @param completed 已完成步骤数
     */
    void finish_failed(const std::shared_ptr<GoalHandlePickTarget> goal_handle, const std::shared_ptr<PickTarget::Result>& result,
        std::int32_t error_code, const std::string& message, std::uint32_t completed) {
        result->success = false;
        result->error_code = cancel_requested_.load() ? kCanceled : error_code;
        result->message = cancel_requested_.load() ? "pick task canceled" : message;
        result->completed_steps = completed;
        result->canceled = cancel_requested_.load() && !motion_pending();
        if(motion_pending()) {
            result->error_code = kMotionFailed;
            result->message = "Motion stop is unconfirmed; new goals remain blocked until the child action terminates";
        }
        result->final_pose = last_pose_;
        if(result->canceled) goal_handle->canceled(result);
        else goal_handle->abort(result);
        executing_.store(false);
    }

private:
    rclcpp_action::Server<PickTarget>::SharedPtr action_server_;  ///< PickTarget Action Server
    rclcpp_action::Client<MoveArm>::SharedPtr motion_client_;     ///< MoveArm capability client
    rclcpp::Client<CommandEef>::SharedPtr eef_client_;            ///< EEF capability client
    std::shared_ptr<PendingMotion> pending_motion_;             ///< 当前 MoveArm 子 goal
    std::mutex active_motion_goal_mutex_;                         ///< 子 goal handle 访问锁
    std::thread execution_thread_;                                ///< 单个 PickTarget 执行线程

    std::atomic_bool executing_{ false };                         ///< 是否正在执行采摘任务
    std::atomic_bool cancel_requested_{ false };                  ///< 当前任务取消标志
    std::atomic_bool shutting_down_{ false };                     ///< 节点是否正在析构
    geometry_msgs::msg::PoseStamped last_pose_;                   ///< 最近一次 MoveArm 返回位姿

    std::string action_name_;                                     ///< PickTarget Action 名称
    std::string motion_action_name_;                              ///< MoveArm Action 名称
    std::string eef_service_name_;                                ///< EEF Service 名称
    double motion_timeout_sec_{ 30.0 };                           ///< 单个运动步骤超时 s
    double service_timeout_sec_{ 3.0 };                           ///< EEF 服务超时 s
    double default_pre_pick_distance_{ 0.15 };                    ///< 默认预采摘距离 m
    double default_approach_distance_{ 0.08 };                    ///< 默认接近距离 m
    double default_retreat_distance_{ 0.10 };                     ///< 默认退出距离 m
    double motion_velocity_scale_{ 0.15 };                        ///< 任务级速度缩放
    double motion_acceleration_scale_{ 0.15 };                    ///< 任务级加速度缩放
};

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

} // namespace tomato_picker_task

/**
 * @brief 启动单目标采摘任务编排节点
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 正常退出返回 0
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tomato_picker_task::PickTaskNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
