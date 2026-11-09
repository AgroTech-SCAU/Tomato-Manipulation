#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace tomato_picker_mujoco {

// Keep the upstream renderer unchanged, but own its ROS node for the entire
// lifetime of its library. Humble callback groups retain weak waitable control
// blocks whose deleters must run before the camera plugin library is unloaded.
class ScopedCameraPlugin final
    : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase {
public:
    using Plugin = mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase;
    ~ScopedCameraPlugin() override { cleanup(); }

    bool init(rclcpp::Node::SharedPtr parent, const mjModel* model, mjData* data) override {
        try {
            std::vector<rclcpp::Parameter> parameters;
            const auto names = parent->list_parameters({"mujoco_plugins.mujoco_camera_plugin"}, 0).names;
            for (const auto& name : names) parameters.push_back(parent->get_parameter(name));
            parameters.emplace_back("use_sim_time", parent->get_parameter("use_sim_time").as_bool());
            rclcpp::NodeOptions options;
            options.context(parent->get_node_base_interface()->get_context());
            options.use_global_arguments(false);
            options.automatically_declare_parameters_from_overrides(true);
            options.parameter_overrides(parameters);
            node_ = std::make_shared<rclcpp::Node>(
                "tomato_picker_mujoco_camera", parent->get_namespace(), options);
            loader_ = std::make_unique<pluginlib::ClassLoader<Plugin>>(
                "mujoco_ros2_control_plugins", "mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase");
            camera_ = loader_->createSharedInstance("mujoco_ros2_control_plugins/CameraPlugin");
            if (!camera_->init(node_->create_sub_node(parent->get_sub_namespace()), model, data)) {
                cleanup();
                return false;
            }
            rclcpp::ExecutorOptions executor_options;
            executor_options.context = options.context();
            executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
            executor_->add_node(node_);
            thread_ = std::thread([this] { executor_->spin(); });
            RCLCPP_INFO(parent->get_logger(), "Camera renderer loaded with owned ROS node lifetime");
            return true;
        } catch (const std::exception& error) {
            RCLCPP_ERROR(parent->get_logger(), "Cannot initialize scoped camera: %s", error.what());
            cleanup();
            return false;
        }
    }

    void update(const mjModel* model, mjData* data) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (camera_) camera_->update(model, data);
    }

    void on_reset(mjData* data) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (camera_) camera_->on_reset(data);
    }

    void cleanup() override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (camera_) camera_->cleanup();
        if (executor_) executor_->cancel();
        if (thread_.joinable()) thread_.join();
        executor_.reset();
        camera_.reset();
        // Destroy the default callback group and its expired weak waitables
        // while the upstream camera's deleter/vtable code is still available.
        node_.reset();
        loader_.reset();
    }

private:
    std::mutex mutex_;
    std::unique_ptr<pluginlib::ClassLoader<Plugin>> loader_;
    std::shared_ptr<Plugin> camera_;
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread thread_;
};
} // namespace tomato_picker_mujoco

PLUGINLIB_EXPORT_CLASS(tomato_picker_mujoco::ScopedCameraPlugin,
    mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
