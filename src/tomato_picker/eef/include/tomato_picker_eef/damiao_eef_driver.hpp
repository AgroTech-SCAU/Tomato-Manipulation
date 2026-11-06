#pragma once

#include "tl/expected.hpp"

#include <atomic>
#include <memory>
#include <string>

namespace tomato_picker_eef {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 真实达妙 EEF Driver 错误
 */
enum class DamiaoEefError {
    NOT_CONFIGURED,
    NOT_ACTIVE,
    INVALID_CONFIG,
    OPEN_FAILED,
    BUS_CONFIG_CONFLICT,
    BUS_TYPE_MISMATCH,
    BUS_PHYSICAL_RESOURCE_CONFLICT,
    ENABLE_FAILED,
    DISABLE_FAILED,
    KP_APR_FAILED,
    MODE_SWITCH_FAILED,
    FEEDBACK_FAILED,
    HOMING_TIMEOUT,
    CANCELLED,
    COMMAND_FAILED,
};

/**
 * @brief EEF 电机反馈
 */
struct EefMotorFeedback {
    double position{ 0.0 };
    double velocity{ 0.0 };
    double torque{ 0.0 };
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 把 Driver 错误转换为稳定日志文本
 * @param error Driver 错误
 * @return 静态错误文本
 */
const char* damiao_eef_error_string(DamiaoEefError error) noexcept;

/**
 * @brief Worker 可独占调用的最小 EEF Driver 接口
 *
 * Worker 通过此接口独占 Driver 的硬件操作
 */
class EefIoDriver {
public:
    virtual ~EefIoDriver() = default;

    /**
     * @brief 配置 EEF Driver
     * @param config_path EEF 配置文件路径
     * @return 成功时返回空值；失败时返回 DamiaoEefError
     */
    virtual tl::expected<void, DamiaoEefError> configure(const std::string& config_path) = 0;

    /**
     * @brief 激活 EEF 并允许 lifecycle 请求取消 homing
     * @param cancel_requested deactivate 或 shutdown 取消标志
     * @return 成功、取消或硬件错误
     */
    virtual tl::expected<void, DamiaoEefError> activate(
        const std::atomic_bool& cancel_requested) = 0;

    /**
     * @brief 刷新 EEF 电机反馈
     * @return 最新反馈或 Driver 错误
     */
    virtual tl::expected<EefMotorFeedback, DamiaoEefError> refresh() = 0;

    /**
     * @brief 发送归一化位置命令
     * @param position_command 归一化位置，范围 [0, 1]
     * @return 成功时返回空值；失败时返回 DamiaoEefError
     */
    virtual tl::expected<void, DamiaoEefError> command_position(double position_command) = 0;

    /**
     * @brief 停止 EEF 运动
     * @return 成功时返回空值；失败时返回 DamiaoEefError
     */
    virtual tl::expected<void, DamiaoEefError> stop() = 0;

    /**
     * @brief 失能 EEF
     * @return 成功时返回空值；失败时返回 DamiaoEefError
     */
    virtual tl::expected<void, DamiaoEefError> deactivate() = 0;

    /**
     * @brief 释放 Driver 持有的底层资源
     */
    virtual void cleanup() noexcept = 0;

    /**
     * @brief 查询 Driver 是否处于激活状态
     * @return 已激活时返回 true
     */
    virtual bool is_active() const noexcept = 0;
};

/**
 * @brief DM4310 EEF 专用 Torque_Pos 设备适配实现
 *
 * 直接复用 SerialArm-Core 安装的 damiao::MotorControl 与共享 CanChannel；
 * 不进入 SerialArm MotorBus/MIT contract
 */
class DamiaoEefDriver final : public EefIoDriver {
public:
    struct Impl;

    /**
     * @brief 构造未配置的 DM4310 EEF Driver
     */
    DamiaoEefDriver();

    /**
     * @brief 安全释放 EEF Driver 资源
     */
    ~DamiaoEefDriver();

    DamiaoEefDriver(const DamiaoEefDriver&) = delete;
    DamiaoEefDriver& operator=(const DamiaoEefDriver&) = delete;

    /**
     * @brief 读取配置并获取 main_can 共享通道
     * @param config_path damiao_eef.yaml 路径
     * @return 配置与通道创建结果
     */
    tl::expected<void, DamiaoEefError> configure(const std::string& config_path) override;

    /**
     * @brief 使能并执行 open-stop homing，最后切换 Torque_Pos
     * @param cancel_requested lifecycle deactivate/shutdown 取消标志
     * @return homing、取消与模式切换结果
     */
    tl::expected<void, DamiaoEefError> activate(
        const std::atomic_bool& cancel_requested) override;

    /**
     * @brief 主动刷新电机 position/velocity/torque
     * @return 最新反馈
     */
    tl::expected<EefMotorFeedback, DamiaoEefError> refresh() override;

    /**
     * @brief 发送归一化位置 Torque_Pos 命令
     * @param position_command 0 为 open、1 为 closed
     * @return 命令发送结果
     */
    tl::expected<void, DamiaoEefError> command_position(double position_command) override;

    /**
     * @brief 刷新当前位置并以 Torque_Pos 保持，失败时安全 disable
     * @return hold 命令结果
     */
    tl::expected<void, DamiaoEefError> stop() override;

    /**
     * @brief 保持当前位置后失能电机
     * @return 停止与失能结果
     */
    tl::expected<void, DamiaoEefError> deactivate() override;

    /**
     * @brief 释放 EEF 通道与低层 Driver
     */
    void cleanup() noexcept override;

    /**
     * @brief 查询是否已完成 homing 且处于 Torque_Pos
     * @return active 状态
     */
    bool is_active() const noexcept override;

private:
    std::unique_ptr<Impl> impl_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace tomato_picker_eef
