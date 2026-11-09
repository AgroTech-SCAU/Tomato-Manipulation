#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <cmath>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "tomato_picker_interfaces/action/move_arm.hpp"
#include "tomato_picker_interfaces/action/move_arm_sequence.hpp"
#include "tomato_picker_interfaces/msg/harvest_physics_state.hpp"

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
    using MoveArmSequence = tomato_picker_interfaces::action::MoveArmSequence;
    using GoalHandleSequence = rclcpp_action::ServerGoalHandle<MoveArmSequence>;

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
    bool plan_joint_fast(moveit::planning_interface::MoveGroupInterface::Plan& plan);
    bool refresh_fast_scene();
    void shortcut_joint_plan(moveit::planning_interface::MoveGroupInterface::Plan& plan,
                             double velocity_scale, double acceleration_scale);
    rclcpp_action::GoalResponse sequence_goal(const rclcpp_action::GoalUUID& uuid,
                                             std::shared_ptr<const MoveArmSequence::Goal> goal);
    void execute_sequence(const std::shared_ptr<GoalHandleSequence> goal_handle);
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

    rclcpp::Subscription<tomato_picker_interfaces::msg::HarvestPhysicsState>::SharedPtr harvest_feedback_;
    std::atomic_uint32_t dropped_fruit_{0};
    std::atomic_uint32_t held_fruit_{0};
    std::vector<double> harvest_recovery_joints_;
    std::string joint_fast_pipeline_, joint_fast_planner_;
    double fast_path_collision_fraction_{.001};
    double joint_shortcut_time_{0.0};
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
    planning_scene::PlanningScenePtr fast_scene_;
    rclcpp_action::Server<MoveArmSequence>::SharedPtr sequence_server_;
    rclcpp::Client<moveit_msgs::srv::GetMotionSequence>::SharedPtr sequence_client_;
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
    const auto pipeline = node_->declare_parameter<std::string>("planning_pipeline", "");
    if(!pipeline.empty()) move_group_->setPlanningPipelineId(pipeline);
    move_group_->setPlanningTime(planning_time_);
    const auto planning_attempts = node_->declare_parameter<int>("planning_attempts", 1);
    if(planning_attempts < 1) throw std::invalid_argument("planning_attempts must be positive");
    move_group_->setNumPlanningAttempts(static_cast<unsigned int>(planning_attempts));
    move_group_->setGoalJointTolerance(node_->declare_parameter<double>(
        "goal_joint_tolerance", move_group_->getGoalJointTolerance()));
    if(!end_effector_link_.empty()) move_group_->setEndEffectorLink(end_effector_link_);

    joint_fast_pipeline_ = node_->declare_parameter<std::string>("joint_fast_pipeline", "");
    joint_fast_planner_ = node_->declare_parameter<std::string>("joint_fast_planner", "PTP");
    fast_path_collision_fraction_ = node_->declare_parameter<double>("fast_path_collision_fraction", .001);
    joint_shortcut_time_ = node_->declare_parameter<double>("joint_shortcut_time", 0.0);
    if(!std::isfinite(fast_path_collision_fraction_) || fast_path_collision_fraction_ <= 0. || fast_path_collision_fraction_ > 1.)
        throw std::invalid_argument("fast_path_collision_fraction must be within (0, 1]");
    if(!joint_fast_pipeline_.empty()) {
        scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
        const auto started = std::chrono::steady_clock::now();
        fast_scene_ = std::make_shared<planning_scene::PlanningScene>(move_group_->getRobotModel());
        RCLCPP_INFO(node_->get_logger(), "Joint collision model prepared in %.3f s",
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
    }

    const auto harvest_topic = node_->declare_parameter<std::string>("harvest_state_topic", "");
    if(!harvest_topic.empty()) {
        harvest_recovery_joints_ = node_->declare_parameter<std::vector<double>>("harvest_recovery_joints", std::vector<double>{});
        const auto joint_count = move_group_->getJointNames().size();
        if(!joint_count || harvest_recovery_joints_.size() % joint_count != 0 ||
            !std::all_of(harvest_recovery_joints_.begin(), harvest_recovery_joints_.end(), [](double value) { return std::isfinite(value); }))
            throw std::invalid_argument("harvest_recovery_joints must contain complete finite joint targets");
        using PhysicsState = tomato_picker_interfaces::msg::HarvestPhysicsState;
        harvest_feedback_ = node_->create_subscription<PhysicsState>(
            harvest_topic, rclcpp::QoS(1).reliable().transient_local(),
            [this](const PhysicsState::SharedPtr message) {
                std::uint32_t dropped = 0;
                if(message->ready) {
                    const auto found = std::find(message->fruit_states.begin(), message->fruit_states.end(), PhysicsState::DROPPED);
                    if(found != message->fruit_states.end()) dropped = static_cast<std::uint32_t>(
                        std::distance(message->fruit_states.begin(), found) + 1);
                }
                held_fruit_.store(message->held_index);
                dropped_fruit_.store(dropped);
            });
    }

    action_server_ = rclcpp_action::create_server<MoveArm>(
        node_,
        action_name_,
        std::bind(&ArmMotionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ArmMotionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ArmMotionServer::handle_accepted, this, std::placeholders::_1));

    const auto sequence_action = node_->declare_parameter<std::string>("sequence_action_name", "");
    if(!sequence_action.empty()) {
        if(!scene_client_) throw std::invalid_argument("sequence_action_name requires joint_fast_pipeline collision validation");
        sequence_client_ = node_->create_client<moveit_msgs::srv::GetMotionSequence>("/plan_sequence_path");
        sequence_server_ = rclcpp_action::create_server<MoveArmSequence>(
            node_, sequence_action, std::bind(&ArmMotionServer::sequence_goal, this, std::placeholders::_1, std::placeholders::_2),
            [this](const std::shared_ptr<GoalHandleSequence>) {
                cancel_requested_.store(true); move_group_->stop();
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandleSequence> handle) {
                if(execution_thread_.joinable()) execution_thread_.join();
                execution_thread_ = std::thread([this, handle]() { execute_sequence(handle); });
            });
        RCLCPP_INFO(node_->get_logger(), "Continuous sequence action enabled: %s", sequence_action.c_str());
    }

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
    bool recovering = !held_fruit_.load() && goal->command_type == MoveArm::Goal::HOME;
    if(!held_fruit_.load() && goal->command_type == MoveArm::Goal::JOINT) {
        const auto names = move_group_->getJointNames();
        const auto& requested = goal->joint_names.empty() ? names : goal->joint_names;
        if(requested.size() == names.size() && goal->joints.size() == names.size()) {
            for(std::size_t offset = 0; offset < harvest_recovery_joints_.size(); offset += names.size()) {
                bool matches = true;
                for(std::size_t i = 0; i < names.size(); ++i) {
                    const auto found = std::find(requested.begin(), requested.end(), names[i]);
                    if(found == requested.end() || std::abs(goal->joints[std::distance(requested.begin(), found)] -
                        harvest_recovery_joints_[offset + i]) > .0001) { matches = false; break; }
                }
                if(matches) { recovering = true; break; }
            }
        }
    }
    const auto dropped_during_motion = [this, recovering]() { return recovering ? 0U : dropped_fruit_.load(); };
    if(const auto dropped = dropped_during_motion()) {
        result->error_code = kExecutionFailed;
        result->message = "Fruit " + std::to_string(dropped) + " dropped; reset the harvest scene before resuming";
        fill_final_state(*result);
        goal_handle->abort(result);
        executing_.store(false);
        return;
    }

    move_group_->setMaxVelocityScalingFactor(normalized_scale(goal->velocity_scale, default_velocity_scale_));
    move_group_->setMaxAccelerationScalingFactor(normalized_scale(goal->acceleration_scale, default_acceleration_scale_));
    move_group_->setPlanningTime(planning_time_);
    move_group_->clearPoseTargets();

    bool target_ready = false;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_started = std::chrono::steady_clock::now();

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
        bool planned = !joint_fast_pipeline_.empty() &&
            (goal->command_type == MoveArm::Goal::HOME || goal->command_type == MoveArm::Goal::JOINT) && plan_joint_fast(plan);
        if(!planned && !cancel_requested_.load() && !shutting_down_.load() && !dropped_during_motion()) {
            planned = move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
            if(planned && joint_shortcut_time_ > 0. &&
               (goal->command_type == MoveArm::Goal::HOME || goal->command_type == MoveArm::Goal::JOINT))
                shortcut_joint_plan(plan, normalized_scale(goal->velocity_scale, default_velocity_scale_),
                                    normalized_scale(goal->acceleration_scale, default_acceleration_scale_));
        }
        if(!planned) {
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

    const double planning_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - planning_started).count();
    const auto& trajectory_points = plan.trajectory_.joint_trajectory.points;
    const double trajectory_seconds = trajectory_points.empty() ? 0.0 :
        rclcpp::Duration(trajectory_points.back().time_from_start).seconds();
    RCLCPP_INFO(node_->get_logger(), "MoveArm plan ready: command=%d, planning=%.3f s, trajectory=%.3f s, points=%zu",
                goal->command_type, planning_seconds, trajectory_seconds, trajectory_points.size());

    if(const auto dropped = dropped_during_motion()) {
        result->error_code = kExecutionFailed;
        result->message = "Fruit " + std::to_string(dropped) + " dropped during planning; motion was not started";
        fill_final_state(*result);
        goal_handle->abort(result);
        executing_.store(false);
        return;
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
        const auto execution_started = std::chrono::steady_clock::now();
        std::atomic_bool finished{false};
        std::thread stop_watch([this, &finished, &dropped_during_motion]() {
            while(!finished.load()) {
                if(cancel_requested_.load() || shutting_down_.load() || dropped_during_motion()) move_group_->stop();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        });
        moveit::core::MoveItErrorCode execution_result;
        try { execution_result = move_group_->execute(plan); }
        catch(...) { finished.store(true); stop_watch.join(); throw; }
        finished.store(true);
        stop_watch.join();
        RCLCPP_INFO(node_->get_logger(), "MoveArm execution finished: command=%d, execution=%.3f s, result=%d",
                    goal->command_type, std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - execution_started).count(), execution_result.val);
        if(execution_result != moveit::core::MoveItErrorCode::SUCCESS || dropped_during_motion()) {
            const bool canceled = cancel_requested_.load() || goal_handle->is_canceling();
            result->error_code = canceled ? kCanceled : kExecutionFailed;
            result->message = dropped_during_motion() ? "Fruit " + std::to_string(dropped_during_motion()) +
                " dropped during transport; arm trajectory stopped" :
                canceled ? "goal canceled during execution" : "MoveIt 2 execution failed";
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

rclcpp_action::GoalResponse ArmMotionServer::sequence_goal(const rclcpp_action::GoalUUID& uuid,
                                                          std::shared_ptr<const MoveArmSequence::Goal> goal) {
    static_cast<void>(uuid);
    if(shutting_down_.load() || goal->request.items.empty() || goal->request.items.size() > 16)
        return rclcpp_action::GoalResponse::REJECT;
    for(const auto& item : goal->request.items) {
        const auto& req = item.req;
        if(req.group_name != planning_group_ || req.pipeline_id != joint_fast_pipeline_ ||
           (req.planner_id != "PTP" && req.planner_id != "LIN") || req.goal_constraints.empty() ||
           !std::isfinite(item.blend_radius) || item.blend_radius < 0. ||
           !std::isfinite(req.max_velocity_scaling_factor) || req.max_velocity_scaling_factor <= 0. || req.max_velocity_scaling_factor > 1. ||
           !std::isfinite(req.max_acceleration_scaling_factor) || req.max_acceleration_scaling_factor <= 0. || req.max_acceleration_scaling_factor > 1.)
            return rclcpp_action::GoalResponse::REJECT;
    }
    if(goal->request.items.back().blend_radius != 0.) return rclcpp_action::GoalResponse::REJECT;
    bool expected = false;
    if(!executing_.compare_exchange_strong(expected, true)) return rclcpp_action::GoalResponse::REJECT;
    cancel_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void ArmMotionServer::execute_sequence(const std::shared_ptr<GoalHandleSequence> goal_handle) {
    auto result = std::make_shared<MoveArmSequence::Result>();
    result->success = false;
    result->error_code = kPlanningFailed;
    const auto interrupted = [this]() { return cancel_requested_.load() || shutting_down_.load() || dropped_fruit_.load(); };
    const auto finish = [&]() {
        try {
            result->final_pose = move_group_->getCurrentPose();
            result->final_joints = move_group_->getCurrentJointValues();
        } catch(const std::exception& error) {
            RCLCPP_WARN(node_->get_logger(), "Sequence final feedback unavailable: %s", error.what());
        }
        if(goal_handle->is_active()) {
            if(goal_handle->is_canceling()) goal_handle->canceled(result);
            else if(result->success) goal_handle->succeed(result);
            else goal_handle->abort(result);
        }
        executing_.store(false);
    };
    const auto feedback = [&](const std::string& stage, float progress) {
        auto value = std::make_shared<MoveArmSequence::Feedback>();
        value->stage = stage; value->progress = progress;
        value->current_pose = move_group_->getCurrentPose();
        goal_handle->publish_feedback(value);
    };
    const auto interruption_reason = [&]() {
        if(const auto fruit = dropped_fruit_.load())
            return "Fruit " + std::to_string(fruit) + " dropped during transport; arm trajectory stopped";
        return std::string("continuous motion canceled");
    };
    try {
        const auto started = std::chrono::steady_clock::now();
        if(interrupted()) { result->message = interruption_reason(); finish(); return; }
        feedback("PLANNING_SEQUENCE", .1F);
        if(!sequence_client_->wait_for_service(std::chrono::seconds(1)))
            throw std::runtime_error("Pilz sequence planning service is unavailable");
        auto request = std::make_shared<moveit_msgs::srv::GetMotionSequence::Request>();
        request->request = goal_handle->get_goal()->request;
        auto future = sequence_client_->async_send_request(request);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
        while(future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
            if(interrupted() || std::chrono::steady_clock::now() >= deadline) {
                sequence_client_->remove_pending_request(future);
                result->message = interrupted() ? interruption_reason() : "continuous motion planning timed out";
                finish(); return;
            }
        }
        const auto response = future.get()->response;
        if(response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS || response.planned_trajectories.size() != 1) {
            result->message = "Pilz sequence planning failed before execution (code=" + std::to_string(response.error_code.val) + ")";
            finish(); return;
        }
        if(interrupted()) { result->message = interruption_reason(); finish(); return; }
        feedback("VALIDATING_SEQUENCE", .3F);
        if(!refresh_fast_scene()) throw std::runtime_error("Complete collision scene unavailable for continuous motion");
        robot_trajectory::RobotTrajectory trajectory(move_group_->getRobotModel(), planning_group_);
        trajectory.setRobotTrajectoryMsg(fast_scene_->getCurrentState(), response.sequence_start, response.planned_trajectories.front());
        if(trajectory.getWayPointCount() < 2) throw std::runtime_error("Continuous motion trajectory is empty");
        const auto* group = move_group_->getRobotModel()->getJointModelGroup(planning_group_);
        const double step = group->getMaximumExtent() * fast_path_collision_fraction_;
        auto sample = trajectory.getFirstWayPoint();
        std::size_t checked = 0;
        for(std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
            const auto& from = trajectory.getWayPoint(i ? i - 1 : 0);
            const auto& to = trajectory.getWayPoint(i);
            const auto segments = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(from.distance(to, group) / step)));
            for(std::size_t j = 0; j <= segments; ++j) {
                if(interrupted()) { result->message = interruption_reason(); finish(); return; }
                from.interpolate(to, static_cast<double>(j) / segments, sample, group);
                sample.update(); ++checked;
                if(!sample.satisfiesBounds(group) || !fast_scene_->isStateValid(sample, planning_group_))
                    throw std::runtime_error("Continuous motion failed final dense collision or joint-bound validation; no segment executed");
            }
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.start_state_ = response.sequence_start;
        plan.trajectory_ = response.planned_trajectories.front();
        RCLCPP_INFO(node_->get_logger(), "Continuous sequence ready: %zu segments, %zu collision samples, planning=%.3f s, trajectory=%.3f s",
                    request->request.items.size(), checked,
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(), trajectory.getDuration());
        if(goal_handle->get_goal()->execute) {
            if(interrupted()) { result->message = interruption_reason(); finish(); return; }
            result->error_code = kExecutionFailed;
            feedback("EXECUTING_SEQUENCE", .6F);
            std::atomic_bool finished{false};
            std::thread stop_watch([this, &finished, &interrupted]() {
                while(!finished.load()) {
                    if(interrupted()) move_group_->stop();
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            });
            moveit::core::MoveItErrorCode status;
            const auto execution_started = std::chrono::steady_clock::now();
            try { status = move_group_->execute(plan); }
            catch(...) { finished.store(true); stop_watch.join(); throw; }
            finished.store(true); stop_watch.join();
            RCLCPP_INFO(node_->get_logger(), "Continuous sequence execution: %.3f s, result=%d",
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - execution_started).count(), status.val);
            if(status != moveit::core::MoveItErrorCode::SUCCESS || interrupted()) {
                result->error_code = kExecutionFailed;
                result->message = interrupted() ? interruption_reason() : "Continuous motion execution failed (code=" + std::to_string(status.val) + ")";
                finish(); return;
            }
        }
        feedback("DONE", 1.F);
        result->success = true; result->error_code = kSuccess;
        result->message = goal_handle->get_goal()->execute ? "continuous motion completed" : "continuous motion planning completed";
    } catch(const std::exception& error) {
        move_group_->stop(); result->message = error.what();
    } catch(...) {
        move_group_->stop(); result->message = "continuous motion failed with an unknown execution error";
    }
    finish();
}

bool ArmMotionServer::plan_joint_fast(moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    using Clock = std::chrono::steady_clock;
    const auto planner_started = Clock::now();
    const auto pipeline = move_group_->getPlanningPipelineId();
    const auto planner = move_group_->getPlannerId();
    moveit::planning_interface::MoveGroupInterface::Plan candidate;
    move_group_->setPlanningPipelineId(joint_fast_pipeline_);
    move_group_->setPlannerId(joint_fast_planner_);
    moveit::core::MoveItErrorCode status;
    try { status = move_group_->plan(candidate); }
    catch(...) {
        move_group_->setPlanningPipelineId(pipeline); move_group_->setPlannerId(planner);
        throw;
    }
    move_group_->setPlanningPipelineId(pipeline); move_group_->setPlannerId(planner);
    const auto planner_finished = Clock::now();
    if(status != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Joint fast path unavailable (code=%d); using original planner", status.val);
        return false;
    }
    // Pilz samples its timed output at 0.1 s. Validate every geometric segment
    // against one complete scene snapshot at the configured OMPL resolution.
    if(!scene_client_->wait_for_service(std::chrono::milliseconds(200))) return false;
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = 1023;
    auto future = scene_client_->async_send_request(request);
    if(future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        scene_client_->remove_pending_request(future); return false;
    }
    const auto snapshot = future.get();
    const auto scene_received = Clock::now();
    const bool cold_scene = !fast_scene_;
    // Retain the robot's FCL geometry between requests; replace all scene data
    // from this request's complete snapshot, including attached bodies and ACM.
    if(!fast_scene_) fast_scene_ = std::make_shared<planning_scene::PlanningScene>(move_group_->getRobotModel());
    if(!fast_scene_->setPlanningSceneMsg(snapshot->scene)) return false;
    const auto scene_updated = Clock::now();
    robot_trajectory::RobotTrajectory trajectory(move_group_->getRobotModel(), planning_group_);
    trajectory.setRobotTrajectoryMsg(fast_scene_->getCurrentState(), candidate.start_state_, candidate.trajectory_);
    const auto* group = move_group_->getRobotModel()->getJointModelGroup(planning_group_);
    const double step = group->getMaximumExtent() * fast_path_collision_fraction_;
    if(trajectory.getWayPointCount() < 2 || step <= 0.) return false;
    auto sample = trajectory.getFirstWayPoint();
    std::size_t checked = 0;
    const auto log_timing = [&](bool accepted) {
        const auto elapsed = [](const auto& begin, const auto& end) {
            return std::chrono::duration<double>(end - begin).count();
        };
        RCLCPP_INFO(node_->get_logger(),
                    "Joint fast path %s: %zu collision samples, trajectory %.3f s; planner=%.3f s, get_scene=%.3f s, scene_%s=%.3f s, validation=%.3f s",
                    accepted ? "accepted" : "rejected", checked, trajectory.getDuration(),
                    elapsed(planner_started, planner_finished), elapsed(planner_finished, scene_received),
                    cold_scene ? "build" : "update", elapsed(scene_received, scene_updated),
                    elapsed(scene_updated, Clock::now()));
    };
    for(std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
        const auto& to = trajectory.getWayPoint(i);
        const auto& from = trajectory.getWayPoint(i ? i - 1 : 0);
        const auto segments = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(from.distance(to, group) / step)));
        for(std::size_t j = 0; j <= segments; ++j) {
            if(cancel_requested_.load() || shutting_down_.load()) return false;
            from.interpolate(to, static_cast<double>(j) / segments, sample, group);
            sample.update();
            ++checked;
            if(!sample.satisfiesBounds(group) || !fast_scene_->isStateValid(sample, planning_group_)) {
                log_timing(false);
                return false;
            }
        }
    }
    log_timing(true);
    plan = std::move(candidate);
    return true;
}

bool ArmMotionServer::refresh_fast_scene() {
    if(!scene_client_ || !scene_client_->wait_for_service(std::chrono::milliseconds(200))) return false;
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = 1023;
    auto future = scene_client_->async_send_request(request);
    if(future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        scene_client_->remove_pending_request(future);
        return false;
    }
    if(!fast_scene_) fast_scene_ = std::make_shared<planning_scene::PlanningScene>(move_group_->getRobotModel());
    return fast_scene_->setPlanningSceneMsg(future.get()->scene);
}

void ArmMotionServer::shortcut_joint_plan(moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                         double velocity_scale, double acceleration_scale) {
    if(!refresh_fast_scene()) return;
    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();
    const auto deadline = started + std::chrono::duration<double>(joint_shortcut_time_);
    const auto* group = move_group_->getRobotModel()->getJointModelGroup(planning_group_);
    const double step = group->getMaximumExtent() * fast_path_collision_fraction_;
    robot_trajectory::RobotTrajectory original(move_group_->getRobotModel(), planning_group_);
    original.setRobotTrajectoryMsg(fast_scene_->getCurrentState(), plan.start_state_, plan.trajectory_);
    if(original.getWayPointCount() < 3 || step <= 0.) return;
    robot_trajectory::RobotTrajectory shortened(move_group_->getRobotModel(), planning_group_);
    shortened.addSuffixWayPoint(original.getFirstWayPoint(), 0.);
    auto sample = original.getFirstWayPoint();
    std::size_t checked = 0, shortcuts = 0, retained_tail = 0;
    const auto valid_segment = [&](std::size_t from_index, std::size_t to_index) {
        const auto& from = original.getWayPoint(from_index);
        const auto& to = original.getWayPoint(to_index);
        const auto segments = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(from.distance(to, group) / step)));
        for(std::size_t j = 0; j <= segments; ++j) {
            if(Clock::now() >= deadline || cancel_requested_.load() || shutting_down_.load()) return false;
            from.interpolate(to, static_cast<double>(j) / segments, sample, group);
            sample.update();
            ++checked;
            if(!sample.satisfiesBounds(group) || !fast_scene_->isStateValid(sample, planning_group_)) return false;
        }
        return true;
    };
    for(std::size_t i = 0; i + 1 < original.getWayPointCount();) {
        std::size_t next = i + 1;
        if(Clock::now() >= deadline && !retained_tail) retained_tail = original.getWayPointCount() - i - 1;
        if(Clock::now() < deadline) {
            for(std::size_t candidate = original.getWayPointCount() - 1; candidate > i + 1;
                candidate = i + (candidate - i) / 2) {
                if(valid_segment(i, candidate)) { next = candidate; ++shortcuts; break; }
                if(Clock::now() >= deadline) break;
            }
        }
        // Unchanged adjacent segments retain the original planner's validation.
        shortened.addSuffixWayPoint(original.getWayPoint(next), 0.);
        i = next;
    }
    if(!shortcuts) return;
    const auto geometric_length = [group](const robot_trajectory::RobotTrajectory& trajectory) {
        double length = 0.;
        for(std::size_t i = 1; i < trajectory.getWayPointCount(); ++i)
            length += trajectory.getWayPoint(i - 1).distance(trajectory.getWayPoint(i), group);
        return length;
    };
    const auto geometric_knots = shortened.getWayPointCount();
    const double original_length = geometric_length(original), shortened_length = geometric_length(shortened);
    // TOTG creates a smooth, densely sampled trajectory instead of handing
    // sparse corners to the controller; verify its final geometry as well.
    trajectory_processing::TimeOptimalTrajectoryGeneration timing(.001, .02);
    if(!timing.computeTimeStamps(shortened, velocity_scale, acceleration_scale)) return;
    if(original.getDuration() - shortened.getDuration() < .02) {
        RCLCPP_INFO(node_->get_logger(),
                    "Joint fallback shortcut retained original: gain %.3f s below 0.020 s; geometric_knots=%zu to %zu, joint_path=%.3f to %.3f, retained_tail=%zu",
                    original.getDuration() - shortened.getDuration(), original.getWayPointCount(), geometric_knots,
                    original_length, shortened_length, retained_tail);
        return;
    }
    for(std::size_t i = 0; i < shortened.getWayPointCount(); ++i) {
        const auto& from = shortened.getWayPoint(i ? i - 1 : 0);
        const auto& to = shortened.getWayPoint(i);
        const auto segments = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(from.distance(to, group) / step)));
        for(std::size_t j = 0; j <= segments; ++j) {
            if(cancel_requested_.load() || shutting_down_.load()) return;
            from.interpolate(to, static_cast<double>(j) / segments, sample, group);
            sample.update();
            ++checked;
            if(!sample.satisfiesBounds(group) || !fast_scene_->isStateValid(sample, planning_group_)) {
                RCLCPP_INFO(node_->get_logger(), "Joint fallback shortcut rejected by final trajectory collision validation");
                return;
            }
        }
    }
    RCLCPP_INFO(node_->get_logger(),
                "Joint fallback shortcut: %zu to %zu points, %.3f to %.3f s, %zu collision samples, optimization=%.3f s; geometric_knots=%zu, joint_path=%.3f to %.3f, retained_tail=%zu",
                original.getWayPointCount(), shortened.getWayPointCount(), original.getDuration(), shortened.getDuration(),
                checked, std::chrono::duration<double>(Clock::now() - started).count(), geometric_knots,
                original_length, shortened_length, retained_tail);
    shortened.getRobotTrajectoryMsg(plan.trajectory_);
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
