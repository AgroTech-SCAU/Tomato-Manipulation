#pragma once

#include "rclcpp/logger.hpp"
#include "tomato_picker_eef/damiao_eef_driver.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace tomato_picker_eef {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief EEF 业务命令，数值与 CommandEef.srv 保持一致
 */
enum class EefCommand : std::uint8_t {
    OPEN = 0,
    CLOSE = 1,
    STOP = 2,
    SET_POSITION = 3,
};

/**
 * @brief Service 命令提交结果
 */
enum class SubmitResult : std::uint8_t {
    ACCEPTED = 0,
    INVALID_COMMAND,
    OUT_OF_RANGE,
    NOT_ACTIVE,
    BUSY,
};

/**
 * @brief 已验证且等待 Driver 执行的 EEF 命令
 */
struct PendingEefCommand {
    EefCommand command{ EefCommand::STOP };  ///< 命令类型
    double value{ 0.0 };                     ///< 归一化位置
};

/**
 * @brief EEF 非实时 Worker 状态
 */
enum class EefWorkerState : std::uint8_t {
    INACTIVE,
    HOMING,
    READY,
    ERROR,
    SHUTDOWN,
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief controller_manager 进程内的单线程 EEF 硬件所有者
 *
 * configure() 在线程启动前建立共享 CanChannel；start() 以后只有 Worker
 * 线程调用 Driver 的硬件方法；ROS callbacks 只提交命令或 lifecycle 请求
 */
class EefWorker final {
public:
    /**
     * @brief 使用真实 Damiao Driver 构造 Worker
     * @param logger Worker 日志器
     */
    explicit EefWorker(rclcpp::Logger logger);

    /**
     * @brief 使用指定 Driver 构造 Worker
     * @param logger Worker 日志器
     * @param driver Worker 独占的 Driver
     */
    EefWorker(rclcpp::Logger logger, std::unique_ptr<EefIoDriver> driver);

    /**
     * @brief 析构 Worker 并等待 I/O 线程退出
     */
    ~EefWorker();

    EefWorker(const EefWorker&) = delete;
    EefWorker& operator=(const EefWorker&) = delete;

    /**
     * @brief 在线程启动前配置 Driver 和 Worker 周期参数
     * @param config_path damiao_eef.yaml 路径
     * @return 成功时返回空值；失败时返回 DamiaoEefError
     */
    tl::expected<void, DamiaoEefError> configure(const std::string& config_path);

    /**
     * @brief 启动非实时 I/O 线程
     * @return 线程启动成功时返回 true
     */
    bool start() noexcept;

    /**
     * @brief 非阻塞请求 homing 和 activate
     */
    void request_activate() noexcept;

    /**
     * @brief 非阻塞请求 stop 和 deactivate
     */
    void request_deactivate() noexcept;

    /**
     * @brief 提交 EEF 业务命令并唤醒 Worker
     * @param command EEF 命令
     * @param value SET_POSITION 归一化位置
     * @return 命令提交结果
     */
    SubmitResult submit(EefCommand command, double value = 0.0) noexcept;

    /**
     * @brief 请求退出并等待 I/O 线程结束
     */
    void shutdown() noexcept;

    /**
     * @brief 获取 Worker 当前状态
     * @return 原子状态快照
     */
    EefWorkerState state() const noexcept;

    /**
     * @brief 查询硬件是否完成 homing 并处于 READY
     * @return READY 时返回 true
     */
    bool hardware_ready() const noexcept;

    /**
     * @brief 查询 Worker 是否接受业务命令
     * @return READY 且可接受命令时返回 true
     */
    bool is_active() const noexcept;

    /**
     * @brief 查询最新归一化 EEF 反馈
     * @return 已有有效反馈时返回位置
     */
    std::optional<double> last_position() const noexcept;

    /**
     * @brief 等待指定状态，供非实时诊断使用
     * @param expected 目标状态
     * @param timeout 最大等待时间
     * @return 在 timeout 前到达目标状态时返回 true
     */
    bool wait_for_state(EefWorkerState expected, std::chrono::milliseconds timeout) const;

private:
    /**
     * @brief Worker 主循环
     */
    void run() noexcept;

    /**
     * @brief 执行 activate 请求
     */
    void handle_activate() noexcept;

    /**
     * @brief 执行 deactivate 请求
     */
    void handle_deactivate() noexcept;

    /**
     * @brief 执行一个 pending command
     * @return 成功或没有 pending command 时返回 true
     */
    bool handle_command() noexcept;

    /**
     * @brief 刷新一次反馈并执行连续失败门限
     * @return 硬件仍可继续运行时返回 true
     */
    bool refresh_feedback() noexcept;

    /**
     * @brief 命令失败或连续反馈失败后的安全失能
     * @param context 错误日志上下文
     * @param error 首个 Driver 错误
     */
    void enter_error(const char* context, DamiaoEefError error) noexcept;

    /**
     * @brief 切换命令接收状态并在失活时清空邮箱
     * @param active 是否接受业务命令
     */
    void set_active(bool active) noexcept;

    /**
     * @brief 取出并清空待执行命令
     * @return 没有 pending 命令时返回空
     */
    std::optional<PendingEefCommand> take_pending() noexcept;

    /**
     * @brief 更新最新归一化反馈
     * @param position 归一化位置，0 为张开，1 为闭合
     */
    void update_feedback(double position) noexcept;

    /**
     * @brief 清空命令邮箱、激活和反馈状态
     */
    void reset_command_state() noexcept;

    /**
     * @brief 原子更新状态并唤醒状态观察者
     * @param state 新状态
     */
    void set_state(EefWorkerState state) noexcept;

private:
    rclcpp::Logger logger_;                       ///< Worker 日志器
    std::unique_ptr<EefIoDriver> driver_;         ///< Worker 独占硬件 Driver
    std::chrono::steady_clock::duration io_period_{ std::chrono::milliseconds(50) };  ///< 反馈周期
    std::size_t max_feedback_failures_{ 3 };      ///< 连续反馈失败安全门限
    std::size_t consecutive_feedback_failures_{ 0 };  ///< 当前连续失败次数
    mutable std::mutex mutex_;                    ///< lifecycle 与命令邮箱互斥
    mutable std::condition_variable cv_;          ///< Worker 与状态观察者通知
    std::optional<PendingEefCommand> pending_;    ///< 最新待执行命令
    std::thread thread_;                          ///< 唯一硬件 I/O 线程
    bool configured_{ false };                    ///< Driver 与 Worker 配置完成
    bool activate_requested_{ false };            ///< lifecycle activate 请求
    bool deactivate_requested_{ false };          ///< lifecycle deactivate 请求
    bool command_requested_{ false };             ///< Service pending 命令通知
    bool shutdown_requested_{ false };            ///< cleanup 或 destructor 退出请求
    std::atomic<EefWorkerState> state_{ EefWorkerState::INACTIVE };  ///< Worker 状态快照
    std::atomic_bool hardware_ready_{ false };     ///< READY 快照
    std::atomic_bool active_{ false };             ///< 是否接受业务命令
    std::atomic_bool has_feedback_{ false };       ///< 是否读到过有效反馈
    std::atomic<double> last_position_{ 0.0 };     ///< 最新归一化反馈
    std::atomic_bool cancel_requested_{ false };             ///< homing lifecycle 取消标志
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace tomato_picker_eef
