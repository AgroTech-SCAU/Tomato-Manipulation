#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "tomato_picker_interfaces/srv/command_eef.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

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
        service_ = create_service<CommandEef>(
            service_name,
            std::bind(&EefMockNode::command_callback, this, std::placeholders::_1, std::placeholders::_2));
        ready_service_ = create_service<Trigger>(
            ready_service_name,
            [](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
                response->success = true;
                response->message = "READY";
            });
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
        std::lock_guard<std::mutex> lock(mutex_);
        switch(request->command) {
            case CommandEef::Request::OPEN:
                position_ = 0.0;
                break;
            case CommandEef::Request::CLOSE:
                position_ = 1.0;
                break;
            case CommandEef::Request::STOP:
                break;
            case CommandEef::Request::SET_POSITION:
                if(!std::isfinite(request->value) || request->value < 0.0 || request->value > 1.0) {
                    response->success = false;
                    response->error_code = kOutOfRange;
                    response->message = "eef position must be within [0, 1]";
                    return;
                }
                position_ = request->value;
                break;
            default:
                response->success = false;
                response->error_code = kInvalidCommand;
                response->message = "unsupported eef command";
                return;
        }

        response->success = true;
        response->error_code = kSuccess;
        response->message = "mock eef command applied";
    }

private:
    rclcpp::Service<CommandEef>::SharedPtr service_;  ///< EEF command Service
    rclcpp::Service<Trigger>::SharedPtr ready_service_;  ///< EEF READY Service
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
    rclcpp::spin(std::make_shared<tomato_picker_eef::EefMockNode>());
    rclcpp::shutdown();
    return 0;
}
