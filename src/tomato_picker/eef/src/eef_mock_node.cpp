#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "tomato_picker_interfaces/srv/command_eef.hpp"

#include <cmath>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <vector>

namespace tomato_picker_eef {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //

namespace {

constexpr std::int32_t kSuccess = 0;          ///< 命令执行成功
constexpr std::int32_t kInvalidCommand = 1;   ///< 命令类型非法
constexpr std::int32_t kOutOfRange = 2;       ///< 位置超出范围

} // namespace

// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //



// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 为无硬件运行提供 CommandEef contract 的纯 ROS mock
 */
class EefMockNode final : public rclcpp::Node {
public:
    using CommandEef = tomato_picker_interfaces::srv::CommandEef;
    using Trigger = std_srvs::srv::Trigger;

    /**
     * @brief 构造 Mock EEF 节点并创建服务
     */
    EefMockNode()
        : Node("eef_mock_node") {
        const auto service_name = declare_parameter<std::string>("service_name", "/tomato_picker/eef/command");
        const auto ready_service_name = declare_parameter<std::string>("ready_service_name", "/tomato_picker/eef/ready");
        position_ = declare_parameter<double>("initial_position", 0.0);
        publish_joints_ = declare_parameter<bool>("publish_joint_states", false);
        travel_time_ = declare_parameter<double>("full_travel_time_sec", 1.2);
        open_positions_ = declare_parameter<std::vector<double>>("gripper_open_positions_m", {0.0, 0.0});
        closed_positions_ = declare_parameter<std::vector<double>>("gripper_closed_positions_m", {-0.007145, -0.007145});
        const auto stop_service_name = declare_parameter<std::string>("stop_service_name", "/tomato_picker/eef/stop");
        if(!std::isfinite(position_) || position_ < 0.0 || position_ > 1.0 ||
           !std::isfinite(travel_time_) || travel_time_ <= 0.0 ||
           open_positions_.size() != 2 || closed_positions_.size() != 2) {
            throw std::invalid_argument("Invalid EEF simulation position or motion parameters");
        }
        for(std::size_t i = 0; i < 2; ++i) {
            if(!std::isfinite(open_positions_[i]) || !std::isfinite(closed_positions_[i]) ||
               open_positions_[i] == closed_positions_[i]) {
                throw std::invalid_argument("EEF simulation endpoints must be finite and distinct");
            }
        }
        service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        service_ = create_service<CommandEef>(
            service_name,
            std::bind(&EefMockNode::command_callback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, service_group_);
        ready_service_ = create_service<Trigger>(
            ready_service_name,
            [](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
                response->success = true;
                response->message = "READY";
            }, rmw_qos_profile_services_default, service_group_);
        stop_service_ = create_service<Trigger>(
            stop_service_name,
            [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
                std::lock_guard<std::mutex> lock(mutex_);
                stop_motion();
                response->success = true;
                response->message = "EEF stopped; no motion is pending";
            }, rmw_qos_profile_services_default, service_group_);
        if(publish_joints_) {
            joints_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
            last_update_ = std::chrono::steady_clock::now();
            timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { update_motion(); });
        }
        RCLCPP_INFO(get_logger(), "EEF mock started; service=%s", service_name.c_str());
    }

private:
    /**
     * @brief 应用一条无硬件 EEF 命令
     * @param request EEF 命令请求
     * @param response EEF 命令结果
     */
    void command_callback(
        const CommandEef::Request::SharedPtr request,
        CommandEef::Response::SharedPtr response) {
        std::unique_lock<std::mutex> lock(mutex_);
        double target = position_;
        switch(request->command) {
            case CommandEef::Request::OPEN:
                target = 0.0;
                break;
            case CommandEef::Request::CLOSE:
                target = 1.0;
                break;
            case CommandEef::Request::STOP:
                stop_motion();
                break;
            case CommandEef::Request::SET_POSITION:
                if(!std::isfinite(request->value) || request->value < 0.0 || request->value > 1.0) {
                    response->success = false;
                    response->error_code = kOutOfRange;
                    response->message = "eef position must be within [0, 1]";
                    return;
                }
                target = request->value;
                break;
            default:
                response->success = false;
                response->error_code = kInvalidCommand;
                response->message = "unsupported eef command";
                return;
        }
        if(request->command != CommandEef::Request::STOP) {
            if(moving_) {
                response->error_code = 3;
                response->message = "EEF is executing another motion";
                return;
            }
            if(!publish_joints_) {
                position_ = target;
            } else {
                target_ = target;
                const auto command_id = ++command_id_;
                moving_ = std::abs(target_ - position_) > 1e-9;
                last_update_ = std::chrono::steady_clock::now();
                while(moving_ && command_id == command_id_ && rclcpp::ok()) {
                    completed_.wait_for(lock, std::chrono::milliseconds(50));
                }
                if(command_id != command_id_ || !rclcpp::ok()) {
                    response->error_code = 4;
                    response->message = "EEF motion interrupted before completion";
                    return;
                }
            }
        }
        response->success = true;
        response->error_code = kSuccess;
        response->message = "simulated eef motion complete";
    }

    // Caller holds mutex_; cancel waiting commands before acknowledging STOP
    void stop_motion() {
        ++command_id_;
        moving_ = false;
        target_ = position_;
        publish_state(0.0);
        completed_.notify_all();
    }

    void publish_state(double velocity) {
        if(!joints_) return;
        sensor_msgs::msg::JointState state;
        state.header.stamp = now();
        state.name = {"joint7", "joint7-1", "joint7-2"};
        const auto left = open_positions_[0] + position_ * (closed_positions_[0] - open_positions_[0]);
        const auto right = open_positions_[1] + position_ * (closed_positions_[1] - open_positions_[1]);
        const auto left_velocity = velocity * (closed_positions_[0] - open_positions_[0]);
        const auto right_velocity = velocity * (closed_positions_[1] - open_positions_[1]);
        // CAD speed ratio gives a 4.3 mm kinematic pinion radius; real motor
        // calibration remains owned by the hardware driver
        state.position = {-left / 0.0043, left, right};
        state.velocity = {-left_velocity / 0.0043, left_velocity, right_velocity};
        joints_->publish(state);
    }

    void update_motion() {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto clock = std::chrono::steady_clock::now();
        const auto dt = std::chrono::duration<double>(clock - last_update_).count();
        last_update_ = clock;
        double velocity = 0.0;
        if(moving_) {
            const auto delta = target_ - position_;
            const auto step = dt / travel_time_;
            if(std::abs(delta) <= step) {
                position_ = target_;
                moving_ = false;
            } else {
                velocity = std::copysign(1.0 / travel_time_, delta);
                position_ += velocity * dt;
            }
        }
        publish_state(velocity);
        if(!moving_) completed_.notify_all();
    }

private:
    rclcpp::Service<CommandEef>::SharedPtr service_;  ///< EEF command Service
    rclcpp::Service<Trigger>::SharedPtr ready_service_;  ///< EEF READY Service
    rclcpp::Service<Trigger>::SharedPtr stop_service_;
    rclcpp::CallbackGroup::SharedPtr service_group_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::condition_variable completed_;
    std::chrono::steady_clock::time_point last_update_;
    std::vector<double> open_positions_, closed_positions_;
    bool publish_joints_{false}, moving_{false};
    double travel_time_{1.2}, target_{0.0};
    std::uint64_t command_id_{0};
    std::mutex mutex_;                                ///< mock 状态互斥
    double position_{ 0.0 };                          ///< 归一化 mock 位置
};

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

} // namespace tomato_picker_eef

/**
 * @brief 启动 Mock EEF capability 节点
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 正常退出返回 0
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    auto node = std::make_shared<tomato_picker_eef::EefMockNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
