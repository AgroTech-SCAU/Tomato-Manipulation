#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <cmath>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "tomato_picker_interfaces/action/move_arm.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace tomato_picker_motion {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //

/**
 * @brief 将 MoveIt 2 规划与执行封装为稳定 MoveArm Action 的能力服务器
 */
class ArmMotionServer final {
public:
    using MoveArm = tomato_picker_interfaces::action::MoveArm;
    using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;

    /**
     * @brief 构造机械臂运动能力服务器
     * @param node 承载 Action Server 与 MoveGroupInterface 的 ROS 2 节点
     */
    explicit ArmMotionServer(const rclcpp::Node::SharedPtr& node);
    /**
     * @brief 停止当前运动任务并回收执行线程
     */
    ~ArmMotionServer();

private:
    /**
     * @brief 校验并接受新的 MoveArm goal
     * @param uuid Action goal UUID
     * @param goal MoveArm goal
     * @return 接受或拒绝策略
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const MoveArm::Goal> goal);
    /**
     * @brief 接受取消请求并停止当前 MoveIt 2 执行
     * @param goal_handle 当前 goal handle
     * @return 取消策略
     */
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveArm> goal_handle);
    /**
     * @brief 接收已接受 goal 并启动独立执行线程
     * @param goal_handle 已接受 goal handle
     */
    void handle_accepted(const std::shared_ptr<GoalHandleMoveArm> goal_handle);
    /**
     * @brief 执行单个 MoveArm goal
     * @param goal_handle 当前 goal handle
     */
    void execute(const std::shared_ptr<GoalHandleMoveArm> goal_handle);
    /**
     * @brief 将 joint goal 写入 MoveGroupInterface
     * @param goal MoveArm goal
     * @return 目标合法并成功写入时返回 true
     */
    bool set_joint_target(const MoveArm::Goal& goal);
    /**
     * @brief 填充 Action 结果中的最终机械臂状态
     * @param result 待写入的结果
     */
    void fill_final_state(MoveArm::Result& result);
    /**
     * @brief 发布当前执行阶段与位姿
     * @param goal_handle 当前 goal handle
     * @param stage 阶段文本
     * @param progress 归一化进度
     */
    void publish_feedback(const std::shared_ptr<GoalHandleMoveArm> goal_handle, const std::string& stage, float progress);

private:
    rclcpp::Node::SharedPtr node_;                                                ///< ROS 2 节点
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;  ///< MoveIt 2 接口
    rclcpp_action::Server<MoveArm>::SharedPtr action_server_;                     ///< MoveArm Action Server

    std::thread execution_thread_;                                               ///< 单个 goal 执行线程
    std::atomic_bool executing_{ false };                                        ///< 是否存在正在执行的 goal
    std::atomic_bool cancel_requested_{ false };                                 ///< 当前 goal 取消标志
    std::atomic_bool shutting_down_{ false };                                    ///< 服务器是否正在析构

    std::string action_name_;                                                    ///< Action 名称
    std::string planning_group_;                                                 ///< MoveIt 规划组名称
    std::string home_named_target_;                                              ///< Home named target
    std::string end_effector_link_;                                              ///< 可选末端 link
    double planning_time_{ 5.0 };                                                ///< 单次规划时间 s
    double default_velocity_scale_{ 0.2 };                                       ///< 默认速度缩放
    double default_acceleration_scale_{ 0.2 };                                   ///< 默认加速度缩放
    double cartesian_eef_step_{ 0.005 };                                         ///< 笛卡尔插值步长 m
    double cartesian_jump_threshold_{ 0.0 };                                     ///< 笛卡尔 jump threshold
    double cartesian_min_fraction_{ 0.95 };                                      ///< LINE 最小可接受轨迹比例
};

namespace {

constexpr std::int32_t kSuccess = 0;              ///< 成功状态码
constexpr std::int32_t kInvalidGoal = 1;          ///< Goal 参数非法
constexpr std::int32_t kPlanningFailed = 2;       ///< MoveIt 2 规划失败
constexpr std::int32_t kExecutionFailed = 3;      ///< MoveIt 2 执行失败
constexpr std::int32_t kCanceled = 4;             ///< Action 被取消

// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

/**
 * @brief 将缩放参数限制到 MoveIt 2 合法范围
 * @param value 请求缩放值
 * @param fallback 非法值回退值
 * @return (0, 1] 范围内缩放值
 */
double normalized_scale(double value, double fallback) {
    if(value <= 0.0) return fallback;
    return std::clamp(value, 0.01, 1.0);
}

/**
 * @brief 从已运行的 move_group 同步 RobotModelLoader 所需参数
 * @param node Motion capability 节点
 */
void sync_moveit_model_parameters(const rclcpp::Node::SharedPtr& node) {
    auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
    if(!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(node->get_logger(), "move_group parameter service is not available; using local MoveIt defaults");
        return;
    }

    const std::vector<std::string> prefixes = {
        "robot_description",
        "robot_description_semantic",
        "robot_description_kinematics",
        "robot_description_planning",
    };
    const auto listed = client->list_parameters(prefixes, 16);
    if(listed.names.empty()) {
        RCLCPP_WARN(node->get_logger(), "move_group exposes no robot model parameters");
        return;
    }

    for(const auto& parameter : client->get_parameters(listed.names)) {
        if(parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) continue;
        if(!node->has_parameter(parameter.get_name())) {
            node->declare_parameter(parameter.get_name(), parameter.get_parameter_value());
        }
    }
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 构造机械臂运动能力服务器
 * @param node 承载 Action Server 与 MoveGroupInterface 的 ROS 2 节点
 */
ArmMotionServer::ArmMotionServer(const rclcpp::Node::SharedPtr& node)
    : node_(node) {
    action_name_ = node_->declare_parameter<std::string>("action_name", "/tomato_picker/motion/move_arm");
    planning_group_ = node_->declare_parameter<std::string>("planning_group", "arm");
    home_named_target_ = node_->declare_parameter<std::string>("home_named_target", "home");
    end_effector_link_ = node_->declare_parameter<std::string>("end_effector_link", "");
    planning_time_ = node_->declare_parameter<double>("planning_time", 5.0);
    default_velocity_scale_ = node_->declare_parameter<double>("velocity_scale", 0.2);
    default_acceleration_scale_ = node_->declare_parameter<double>("acceleration_scale", 0.2);
    cartesian_eef_step_ = node_->declare_parameter<double>("cartesian_eef_step", 0.005);
    cartesian_jump_threshold_ = node_->declare_parameter<double>("cartesian_jump_threshold", 0.0);
    cartesian_min_fraction_ = node_->declare_parameter<double>("cartesian_min_fraction", 0.95);

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_);
    move_group_->setPlanningTime(planning_time_);
    const auto planning_attempts = node_->declare_parameter<int>("planning_attempts", 1);
    if(planning_attempts < 1) throw std::invalid_argument("planning_attempts must be positive");
    move_group_->setNumPlanningAttempts(static_cast<unsigned int>(planning_attempts));
    move_group_->setGoalJointTolerance(node_->declare_parameter<double>(
        "goal_joint_tolerance", move_group_->getGoalJointTolerance()));
    if(!end_effector_link_.empty()) move_group_->setEndEffectorLink(end_effector_link_);

    action_server_ = rclcpp_action::create_server<MoveArm>(
        node_,
        action_name_,
        std::bind(&ArmMotionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ArmMotionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ArmMotionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "ArmMotionServer started; action=%s planning_group=%s", action_name_.c_str(), planning_group_.c_str());
}

/**
 * @brief 停止当前运动任务并回收执行线程
 */
ArmMotionServer::~ArmMotionServer() {
    shutting_down_.store(true);
    cancel_requested_.store(true);
    try {
        move_group_->stop();
    }
    catch(const std::exception& error) {
        RCLCPP_WARN(node_->get_logger(), "Failed to stop MoveIt execution during shutdown: %s", error.what());
    }
    if(execution_thread_.joinable()) execution_thread_.join();
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 校验并接受新的 MoveArm goal
 * @param uuid Action goal UUID
 * @param goal MoveArm goal
 * @return 接受或拒绝策略
 */
rclcpp_action::GoalResponse ArmMotionServer::handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const MoveArm::Goal> goal) {
    static_cast<void>(uuid);
    if(shutting_down_.load()) return rclcpp_action::GoalResponse::REJECT;
    if(!std::isfinite(goal->velocity_scale) || !std::isfinite(goal->acceleration_scale))
        return rclcpp_action::GoalResponse::REJECT;
    if(goal->command_type == MoveArm::Goal::POSE || goal->command_type == MoveArm::Goal::LINE) {
        const auto& p = goal->target_pose.pose.position;
        const auto& q = goal->target_pose.pose.orientation;
        const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm) || std::abs(norm-1.0) > 0.01)
            return rclcpp_action::GoalResponse::REJECT;
    }
    for(double value : goal->joints) if(!std::isfinite(value)) return rclcpp_action::GoalResponse::REJECT;
    if(goal->command_type > MoveArm::Goal::LINE) return rclcpp_action::GoalResponse::REJECT;
    if(goal->command_type == MoveArm::Goal::JOINT && goal->joints.empty()) return rclcpp_action::GoalResponse::REJECT;
    if(goal->command_type == MoveArm::Goal::POSE && goal->target_pose.header.frame_id.empty()) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    if(goal->command_type == MoveArm::Goal::LINE) {
        const auto& target_frame = goal->target_pose.header.frame_id;
        const auto reference_frame = move_group_->getPoseReferenceFrame();
        if(target_frame.empty()) {
            RCLCPP_WARN(node_->get_logger(), "LINE target frame is empty; expected MoveGroup reference frame '%s'", reference_frame.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if(target_frame != reference_frame) {
            RCLCPP_WARN(node_->get_logger(), "LINE target frame '%s' does not match MoveGroup reference frame '%s'",
                target_frame.c_str(), reference_frame.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    bool expected = false;
    if(!executing_.compare_exchange_strong(expected, true)) return rclcpp_action::GoalResponse::REJECT;
    cancel_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief 接受取消请求并停止当前 MoveIt 2 执行
 * @param goal_handle 当前 goal handle
 * @return 取消策略
 */
rclcpp_action::CancelResponse ArmMotionServer::handle_cancel(const std::shared_ptr<GoalHandleMoveArm> goal_handle) {
    static_cast<void>(goal_handle);
    cancel_requested_.store(true);
    move_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief 接收已接受 goal 并启动独立执行线程
 * @param goal_handle 已接受 goal handle
 */
void ArmMotionServer::handle_accepted(const std::shared_ptr<GoalHandleMoveArm> goal_handle) {
    if(execution_thread_.joinable()) execution_thread_.join();
    execution_thread_ = std::thread([this, goal_handle]() {
        try { execute(goal_handle); }
        catch(const std::exception& error) {
            move_group_->stop();
            auto result = std::make_shared<MoveArm::Result>();
            result->success = false;
            result->error_code = kExecutionFailed;
            result->message = error.what();
            if(goal_handle->is_active()) goal_handle->abort(result);
            executing_.store(false);
        }
    });
}

/**
 * @brief 执行单个 MoveArm goal
 * @param goal_handle 当前 goal handle
 */
void ArmMotionServer::execute(const std::shared_ptr<GoalHandleMoveArm> goal_handle) {

    auto result = std::make_shared<MoveArm::Result>();
    result->success = false;
    result->error_code = kInvalidGoal;

    const auto goal = goal_handle->get_goal();

    move_group_->setMaxVelocityScalingFactor(normalized_scale(goal->velocity_scale, default_velocity_scale_));
    move_group_->setMaxAccelerationScalingFactor(normalized_scale(goal->acceleration_scale, default_acceleration_scale_));
    move_group_->setPlanningTime(planning_time_);
    move_group_->clearPoseTargets();

    bool target_ready = false;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    publish_feedback(goal_handle, "SET_TARGET", 0.1F);
    switch(goal->command_type) {
        case MoveArm::Goal::HOME:
            target_ready = move_group_->setNamedTarget(home_named_target_);
            break;
        case MoveArm::Goal::JOINT:
            target_ready = set_joint_target(*goal);
            break;
        case MoveArm::Goal::POSE:
            target_ready = move_group_->setPoseTarget(goal->target_pose);
            break;
        case MoveArm::Goal::LINE: {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(goal->target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            publish_feedback(goal_handle, "CARTESIAN_PLAN", 0.3F);
            const double fraction = move_group_->computeCartesianPath(waypoints, cartesian_eef_step_, cartesian_jump_threshold_, trajectory, true);
            if(!std::isfinite(fraction) || fraction < cartesian_min_fraction_) {
                const bool canceled = cancel_requested_.load() || goal_handle->is_canceling();
                result->error_code = canceled ? kCanceled : kPlanningFailed;
                result->message = canceled ? "goal canceled during cartesian planning" : "cartesian path fraction below threshold";
                fill_final_state(*result);
                if(canceled) goal_handle->canceled(result);
                else goal_handle->abort(result);
                executing_.store(false);
                return;
            }
            auto state = move_group_->getCurrentState(1.0);
            if(!state) throw std::runtime_error("Current robot state unavailable for Cartesian timing");
            robot_trajectory::RobotTrajectory timed(move_group_->getRobotModel(), planning_group_);
            timed.setRobotTrajectoryMsg(*state, trajectory);
            trajectory_processing::IterativeParabolicTimeParameterization timing;
            if(!timing.computeTimeStamps(timed,
                    normalized_scale(goal->velocity_scale, default_velocity_scale_),
                    normalized_scale(goal->acceleration_scale, default_acceleration_scale_)))
                throw std::runtime_error("Cartesian trajectory timing failed");
            timed.getRobotTrajectoryMsg(trajectory);
            plan.trajectory_ = trajectory;
            target_ready = true;
            break;
        }
        default:
            target_ready = false;
            break;
    }

    if(!target_ready) {
        result->message = "invalid or unsupported target";
        fill_final_state(*result);
        goal_handle->abort(result);
        executing_.store(false);
        return;
    }

    if(goal->command_type != MoveArm::Goal::LINE) {
        publish_feedback(goal_handle, "PLANNING", 0.4F);
        if(move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            const bool canceled = cancel_requested_.load() || goal_handle->is_canceling();
            result->error_code = canceled ? kCanceled : kPlanningFailed;
            result->message = canceled ? "goal canceled during planning" : "MoveIt 2 planning failed";
            fill_final_state(*result);
            if(canceled) goal_handle->canceled(result);
            else goal_handle->abort(result);
            executing_.store(false);
            return;
        }
    }

    if(cancel_requested_.load() || goal_handle->is_canceling()) {
        result->error_code = kCanceled;
        result->message = "goal canceled before execution";
        result->success = false;
        fill_final_state(*result);
        goal_handle->canceled(result);
        executing_.store(false);
        return;
    }

    if(goal->execute) {
        publish_feedback(goal_handle, "EXECUTING", 0.7F);
        std::atomic_bool finished{false};
        std::thread stop_watch([this, &finished]() {
            while(!finished.load()) {
                if(cancel_requested_.load() || shutting_down_.load()) move_group_->stop();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        });
        moveit::core::MoveItErrorCode execution_result;
        try { execution_result = move_group_->execute(plan); }
        catch(...) { finished.store(true); stop_watch.join(); throw; }
        finished.store(true);
        stop_watch.join();
        if(execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
            const bool canceled = cancel_requested_.load() || goal_handle->is_canceling();
            result->error_code = canceled ? kCanceled : kExecutionFailed;
            result->message = canceled ? "goal canceled during execution" : "MoveIt 2 execution failed";
            fill_final_state(*result);
            if(canceled) goal_handle->canceled(result);
            else goal_handle->abort(result);
            executing_.store(false);
            return;
        }
    }

    if(cancel_requested_.load() || goal_handle->is_canceling()) {
        result->error_code = kCanceled;
        result->message = "goal canceled";
        fill_final_state(*result);
        goal_handle->canceled(result);
        executing_.store(false);
        return;
    }

    publish_feedback(goal_handle, "DONE", 1.0F);
    result->success = true;
    result->error_code = kSuccess;
    result->message = goal->execute ? "motion completed" : "planning completed";
    fill_final_state(*result);
    goal_handle->succeed(result);
    executing_.store(false);
}

/**
 * @brief 将 joint goal 写入 MoveGroupInterface
 * @param goal MoveArm goal
 * @return 目标合法并成功写入时返回 true
 */
bool ArmMotionServer::set_joint_target(const MoveArm::Goal& goal) {
    if(goal.joints.empty()) return false;

    if(goal.joint_names.empty()) {
        const auto current = move_group_->getCurrentJointValues();
        if(current.size() != goal.joints.size()) return false;
        return move_group_->setJointValueTarget(goal.joints);
    }
    if(goal.joint_names.size() != goal.joints.size()) return false;

    const auto current_values = move_group_->getCurrentJointValues();
    const auto joint_names = move_group_->getJointNames();
    if(current_values.size() != joint_names.size()) return false;
    if(!move_group_->setJointValueTarget(current_values)) return false;

    std::map<std::string, double> target;
    for(std::size_t i = 0; i < goal.joint_names.size(); ++i) {
        if(std::find(joint_names.begin(), joint_names.end(), goal.joint_names[i]) == joint_names.end()) return false;
        target.emplace(goal.joint_names[i], goal.joints[i]);
    }
    return move_group_->setJointValueTarget(target);
}

/**
 * @brief 填充 Action 结果中的最终机械臂状态
 * @param result 待写入的结果
 */
void ArmMotionServer::fill_final_state(MoveArm::Result& result) {
    result.final_pose = move_group_->getCurrentPose();
    result.final_joints = move_group_->getCurrentJointValues();
}

/**
 * @brief 发布当前执行阶段与位姿
 * @param goal_handle 当前 goal handle
 * @param stage 阶段文本
 * @param progress 归一化进度
 */
void ArmMotionServer::publish_feedback(const std::shared_ptr<GoalHandleMoveArm> goal_handle, const std::string& stage, float progress) {
    auto feedback = std::make_shared<MoveArm::Feedback>();
    feedback->stage = stage;
    feedback->progress = progress;
    feedback->current_pose = move_group_->getCurrentPose();
    goal_handle->publish_feedback(feedback);
}

} // namespace tomato_picker_motion

/**
 * @brief 启动 MoveArm capability 节点
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 正常退出返回 0
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("arm_motion_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false));
    tomato_picker_motion::sync_moveit_model_parameters(node);
    auto server = std::make_shared<tomato_picker_motion::ArmMotionServer>(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    executor.remove_node(node);
    server.reset();
    rclcpp::shutdown();
    return 0;
}
