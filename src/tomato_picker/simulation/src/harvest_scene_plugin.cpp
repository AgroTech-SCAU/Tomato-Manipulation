#include <mujoco/mujoco.h>
#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tomato_picker_interfaces/msg/harvest_physics_state.hpp>
#include <tomato_picker_interfaces/srv/command_eef.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace tomato_picker_mujoco {

// Scene services queue mutations for the physics thread. Fruit positions are
// written only on RESET; transport and release are solved by MuJoCo contacts.
class HarvestScenePlugin final
    : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase {
public:
    using PhysicsState = tomato_picker_interfaces::msg::HarvestPhysicsState;
    using CommandEef = tomato_picker_interfaces::srv::CommandEef;
    using Trigger = std_srvs::srv::Trigger;

    ~HarvestScenePlugin() override { cleanup(); }

    bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData*) override {
        node_ = std::move(node);
        model_ = model;
        try {
            const auto prefix = "mujoco_plugins." + node_->get_sub_namespace() + ".";
            offsets_ = parameter<std::vector<double>>(prefix + "fruit_offsets_m",
                {-.055, 0., .10, 0., 0., .12, .055, 0., .10});
            world_frame_ = parameter<std::string>(prefix + "world_frame", "base_link");
            radius_ = positive_parameter(prefix + "fruit_radius_m", .025);
            closed_position_ = parameter<double>(prefix + "closed_position_m", -.008);
            open_position_ = parameter<double>(prefix + "open_position_m", 0.);
            position_tolerance_ = positive_parameter(prefix + "position_tolerance_m", .0002);
            velocity_tolerance_ = positive_parameter(prefix + "velocity_tolerance_mps", .003);
            force_min_ = positive_parameter(prefix + "grasp_force_min_n", .02);
            grasp_stable_ = positive_parameter(prefix + "grasp_stable_sec", .12);
            settle_speed_ = positive_parameter(prefix + "settle_speed_mps", .04);
            settle_stable_ = positive_parameter(prefix + "settle_stable_sec", .25);
            service_timeout_ = positive_parameter(prefix + "service_timeout_sec", 12.);
            publish_period_ = 1. / positive_parameter(prefix + "publish_rate_hz", 50.);
            if (offsets_.empty() || offsets_.size() % 3 ||
                !std::all_of(offsets_.begin(), offsets_.end(), [](double v) { return std::isfinite(v); }) ||
                !std::isfinite(closed_position_) || !std::isfinite(open_position_) ||
                closed_position_ >= open_position_) {
                throw std::runtime_error("Invalid fruit offsets or gripper endpoints");
            }
            tip_ = named(mjOBJ_SITE, "tip");
            floor_ = named(mjOBJ_GEOM, "basket_floor");
            finger_bodies_ = {named(mjOBJ_BODY, "Link7-1"), named(mjOBJ_BODY, "Link7-2")};
            for (std::size_t i = 0; i < 2; ++i) {
                const auto joint = named(mjOBJ_JOINT, i == 0 ? "joint7-1" : "joint7-2");
                finger_qpos_[i] = model_->jnt_qposadr[joint];
                finger_dof_[i] = model_->jnt_dofadr[joint];
            }
            fruits_.resize(offsets_.size() / 3);
            for (std::size_t i = 0; i < fruits_.size(); ++i) {
                auto& fruit = fruits_[i];
                const auto suffix = std::to_string(i + 1);
                fruit.body = named(mjOBJ_BODY, "fruit_" + suffix);
                fruit.geom = named(mjOBJ_GEOM, "fruit_geom_" + suffix);
                fruit.weld = named(mjOBJ_EQUALITY, "stem_weld_" + suffix);
                const auto stem = named(mjOBJ_BODY, "stem_" + suffix);
                fruit.mocap = model_->body_mocapid[stem];
                const auto joint = model_->body_jntadr[fruit.body];
                if (fruit.mocap < 0 || model_->body_jntnum[fruit.body] != 1 ||
                    joint < 0 || model_->jnt_type[joint] != mjJNT_FREE) {
                    throw std::runtime_error("Each fruit must have one free joint and a mocap stem");
                }
                fruit.qpos = model_->jnt_qposadr[joint];
                fruit.dof = model_->jnt_dofadr[joint];
            }
            // The host unloads plugins before destroying its node. Keep our
            // services and publisher-event waitables out of its default group.
            group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant, false);
            rclcpp::PublisherOptions publisher_options;
            publisher_options.callback_group = group_;
            publisher_ = node_->create_publisher<PhysicsState>(
                "/simulation/physics/state", rclcpp::QoS(1).reliable().transient_local(), publisher_options);
            command_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/gripper_controller/commands", 10, publisher_options);
            for (const auto& item : std::array<std::pair<const char*, SceneCommand>, 4>{{
                    {"reset", SceneCommand::Reset}, {"select_next", SceneCommand::Select},
                    {"grasp", SceneCommand::Grasp}, {"release", SceneCommand::Release}}}) {
                scene_services_.push_back(node_->create_service<Trigger>(
                    std::string("/simulation/physics/") + item.first,
                    [this, command = item.second](const Trigger::Request::SharedPtr,
                                                 Trigger::Response::SharedPtr response) {
                        scene_service(command, response);
                    }, rmw_qos_profile_services_default, group_));
            }
            eef_service_ = node_->create_service<CommandEef>(
                "/tomato_picker/eef/command",
                [this](const CommandEef::Request::SharedPtr request, CommandEef::Response::SharedPtr response) {
                    eef_service(request, response);
                }, rmw_qos_profile_services_default, group_);
            stop_service_ = node_->create_service<Trigger>(
                "/tomato_picker/eef/stop",
                [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
                    std::unique_lock<std::mutex> lock(mutex_);
                    response->success = confirm_stop(lock, response->message);
                }, rmw_qos_profile_services_default, group_);
            ready_service_ = node_->create_service<Trigger>(
                "/tomato_picker/eef/ready",
                [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    response->success = have_feedback_ && !shutdown_;
                    response->message = response->success ? "READY" : "Waiting for physics feedback";
                }, rmw_qos_profile_services_default, group_);
            rclcpp::ExecutorOptions executor_options;
            executor_options.context = node_->get_node_base_interface()->get_context();
            service_executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(executor_options, 3);
            service_executor_->add_callback_group(group_, node_->get_node_base_interface());
            service_thread_ = std::thread([this] { service_executor_->spin(); });
            RCLCPP_INFO(node_->get_logger(), "Contact harvesting plugin loaded for %zu fruits", fruits_.size());
            return true;
        } catch (const std::exception& error) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot initialize harvest physics: %s", error.what());
            return false;
        }
    }

    void pre_step(mjData* data) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (shutdown_) return;
        have_feedback_ = true;
        for (std::size_t i = 0; i < 2; ++i) {
            finger_positions_[i] = data->qpos[finger_qpos_[i]];
            finger_velocities_[i] = data->qvel[finger_dof_[i]];
        }
        if (stopping_) {
            if (fingers_still()) {
                if (stop_still_since_ < 0.) stop_still_since_ = data->time;
                if (data->time - stop_still_since_ >= .08) {
                    stop_confirmed_ = true;
                    completed_.notify_all();
                }
            } else {
                stop_still_since_ = -1.;
            }
        }
        contact_snapshot(data);
        for (auto& fruit : fruits_) {
            if (fruit.left_force >= force_min_ && fruit.right_force >= force_min_) {
                if (fruit.contact_since < 0.) fruit.contact_since = data->time;
                fruit.last_contact = data->time;
            } else {
                fruit.contact_since = -1.;
            }
        }
        if (pending_scene_ != SceneCommand::None && !scene_applied_) apply_scene_command(data);
        if (initialized_) track_fruits(data);
        update_motion(data);
    }

    void update(const mjModel*, mjData* data) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (shutdown_ || data->time < required_snapshot_time_) return;
        if (data->time - last_publish_time_ < publish_period_ && published_revision_ == revision_) return;
        last_publish_time_ = data->time;
        PhysicsState message;
        message.header.stamp = node_->now();
        message.header.frame_id = world_frame_;
        message.ready = initialized_;
        message.revision = revision_;
        message.phase = phase_;
        message.selected_index = selected_ + 1;
        message.held_index = held_ + 1;
        message.initial_tcp = initial_tcp_;
        for (const auto& fruit : fruits_) {
            geometry_msgs::msg::Pose pose;
            const auto* p = data->xpos + 3 * fruit.body;
            const auto* q = data->xquat + 4 * fruit.body;
            pose.position.x = p[0]; pose.position.y = p[1]; pose.position.z = p[2];
            pose.orientation.w = q[0]; pose.orientation.x = q[1];
            pose.orientation.y = q[2]; pose.orientation.z = q[3];
            message.fruit_poses.push_back(pose);
            mjtNum velocity[6];
            mj_objectVelocity(model_, data, mjOBJ_BODY, fruit.body, velocity, 0);
            geometry_msgs::msg::Twist twist;
            twist.angular.x = velocity[0]; twist.angular.y = velocity[1]; twist.angular.z = velocity[2];
            twist.linear.x = velocity[3]; twist.linear.y = velocity[4]; twist.linear.z = velocity[5];
            message.fruit_twists.push_back(twist);
            message.fruit_states.push_back(fruit.state);
            message.left_contacts.push_back(fruit.left_force >= force_min_);
            message.right_contacts.push_back(fruit.right_force >= force_min_);
            message.left_forces.push_back(fruit.left_force);
            message.right_forces.push_back(fruit.right_force);
            if (fruit.state == PhysicsState::COLLECTED) ++message.harvested_count;
        }
        publisher_->publish(message);
        published_revision_ = revision_;
        completed_.notify_all();
    }

    void on_reset(mjData*) override {
        std::lock_guard<std::mutex> lock(mutex_);
        reset_metadata();
        stop_motion("World reset interrupted EEF motion");
        if (pending_scene_ != SceneCommand::None) finish_scene(false, "World reset interrupted scene operation");
        required_snapshot_time_ = -1.;
        last_publish_time_ = -1.;
        ++revision_;
    }

    void cleanup() override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shutdown_) return;
            shutdown_ = true;
            motion_running_ = false;
            completed_.notify_all();
        }
        // Waiting service callbacks observe shutdown_ and leave while their
        // code is still loaded. Joining also drains already-selected callbacks.
        if (service_executor_) service_executor_->cancel();
        if (service_thread_.joinable()) service_thread_.join();
        service_executor_.reset();
        scene_services_.clear();
        eef_service_.reset();
        stop_service_.reset();
        ready_service_.reset();
        publisher_.reset();
        command_publisher_.reset();
        group_.reset();
        node_.reset();
    }

private:
    enum class SceneCommand { None, Reset, Select, Grasp, Release };
    struct Fruit {
        int body{-1}, geom{-1}, weld{-1}, mocap{-1}, qpos{-1}, dof{-1};
        std::uint8_t state{PhysicsState::AVAILABLE};
        double left_force{0.}, right_force{0.}, contact_since{-1.}, last_contact{-1.}, settle_since{-1.};
        bool floor_contact{false};
    };

    template<class T> T parameter(const std::string& name, const T& fallback) {
        if (!node_->has_parameter(name)) node_->declare_parameter<T>(name, fallback);
        return node_->get_parameter(name).get_value<T>();
    }
    double positive_parameter(const std::string& name, double fallback) {
        const auto value = parameter<double>(name, fallback);
        if (!std::isfinite(value) || value <= 0.) throw std::runtime_error(name + " must be finite and positive");
        return value;
    }
    int named(mjtObj kind, const std::string& name) const {
        const auto id = mj_name2id(model_, kind, name.c_str());
        if (id < 0) throw std::runtime_error("MJCF is missing " + name);
        return id;
    }
    bool finger_geom(int geom, std::size_t side) const {
        auto body = model_->geom_bodyid[geom];
        while (body > 0) {
            if (body == finger_bodies_[side]) return true;
            body = model_->body_parentid[body];
        }
        return false;
    }
    void contact_snapshot(const mjData* data) {
        for (auto& fruit : fruits_) {
            fruit.left_force = fruit.right_force = 0.;
            fruit.floor_contact = false;
        }
        for (int i = 0; i < data->ncon; ++i) {
            const auto& contact = data->contact[i];
            if (contact.efc_address < 0) continue;
            mjtNum force[6];
            mj_contactForce(model_, data, i, force);
            if (!std::isfinite(force[0]) || force[0] <= 0.) continue;
            for (auto& fruit : fruits_) {
                const auto other = contact.geom[0] == fruit.geom ? contact.geom[1] :
                                   contact.geom[1] == fruit.geom ? contact.geom[0] : -1;
                if (other < 0) continue;
                if (finger_geom(other, 0)) fruit.left_force += force[0];
                if (finger_geom(other, 1)) fruit.right_force += force[0];
                if (other == floor_) fruit.floor_contact = true;
            }
        }
    }
    bool fingers_still() const {
        return std::abs(finger_velocities_[0]) <= velocity_tolerance_ &&
               std::abs(finger_velocities_[1]) <= velocity_tolerance_;
    }
    bool fingers_at(double target) const {
        return std::abs(finger_positions_[0] - target) <= position_tolerance_ &&
               std::abs(finger_positions_[1] - target) <= position_tolerance_ && fingers_still();
    }
    bool stable_contact(int index, double now) const {
        return index >= 0 && fruits_[index].contact_since >= 0. &&
               now - fruits_[index].contact_since >= grasp_stable_ && fingers_still();
    }
    void publish_target(double target) {
        std_msgs::msg::Float64MultiArray command;
        command.data = {target};
        command_publisher_->publish(command);
    }
    void stop_motion(const std::string& reason) {
        if (motion_running_) {
            motion_result_id_ = motion_id_;
            motion_success_ = false;
            motion_message_ = reason;
        }
        motion_running_ = false;
        if (have_feedback_) publish_target(finger_positions_[0]);
        completed_.notify_all();
    }
    bool confirm_stop(std::unique_lock<std::mutex>& lock, std::string& message) {
        if (shutdown_ || !have_feedback_) { message = "No physics feedback to confirm STOP"; return false; }
        if (stopping_) { message = "An EEF STOP is already awaiting physical confirmation"; return false; }
        stopping_ = true;
        stop_confirmed_ = false;
        stop_still_since_ = -1.;
        stop_motion("EEF motion interrupted by STOP");
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration<double>(std::min(service_timeout_, 5.));
        completed_.wait_until(lock, deadline, [this] { return shutdown_ || stop_confirmed_; });
        const bool success = !shutdown_ && stop_confirmed_;
        stopping_ = false;
        message = success ? "EEF STOP confirmed by stable measured finger feedback" :
                            "EEF hold was sent, but stable physical STOP feedback was not confirmed";
        return success;
    }
    void update_motion(const mjData* data) {
        if (!motion_running_) return;
        bool reached = fingers_at(motion_target_);
        if (motion_grasp_index_ >= 0) reached = stable_contact(motion_grasp_index_, data->time);
        if (!reached) return;
        motion_running_ = false;
        motion_result_id_ = motion_id_;
        motion_success_ = true;
        motion_message_ = motion_grasp_index_ >= 0 ? "Both fingers established stable physical contact" :
                                                    "Measured gripper motion complete";
        completed_.notify_all();
    }
    void eef_service(const CommandEef::Request::SharedPtr& request,
                     const CommandEef::Response::SharedPtr& response) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (shutdown_ || !have_feedback_) {
            response->error_code = 3; response->message = "No physics feedback"; return;
        }
        if (request->command == CommandEef::Request::STOP) {
            response->success = confirm_stop(lock, response->message);
            response->error_code = response->success ? 0 : 4;
            return;
        }
        double fraction = 0.;
        if (request->command == CommandEef::Request::CLOSE) fraction = 1.;
        else if (request->command == CommandEef::Request::SET_POSITION) fraction = request->value;
        else if (request->command != CommandEef::Request::OPEN) {
            response->error_code = 1; response->message = "Unsupported EEF command"; return;
        }
        if (!std::isfinite(fraction) || fraction < 0. || fraction > 1.) {
            response->error_code = 2; response->message = "EEF position must be within [0, 1]"; return;
        }
        if (motion_service_busy_ || stopping_) {
            response->error_code = 3; response->message = "EEF is executing another motion"; return;
        }
        motion_service_busy_ = true;
        const auto id = ++motion_id_;
        motion_target_ = open_position_ + fraction * (closed_position_ - open_position_);
        motion_grasp_index_ = fraction >= .99 ? selected_ : -1;
        opening_held_fruit_ = fraction <= .01 && held_ >= 0;
        motion_running_ = true;
        publish_target(motion_target_);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(service_timeout_);
        if (!completed_.wait_until(lock, deadline, [this, id] { return shutdown_ || motion_result_id_ >= id; })) {
            stop_motion("EEF physical motion timed out before reaching its completion condition");
        }
        response->success = !shutdown_ && motion_result_id_ == id && motion_success_;
        response->error_code = response->success ? 0 : 4;
        response->message = shutdown_ ? "Physics backend stopped" : motion_message_;
        motion_service_busy_ = false;
    }

    void scene_service(SceneCommand command, const Trigger::Response::SharedPtr& response) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (shutdown_ || !have_feedback_) {
            response->message = "Waiting for physics feedback"; return;
        }
        if (scene_service_busy_) {
            response->message = "Another harvest operation is awaiting physical completion"; return;
        }
        scene_service_busy_ = true;
        pending_scene_ = command;
        scene_applied_ = false;
        const auto id = ++scene_id_;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(service_timeout_);
        const auto done = [this, id] {
            return shutdown_ || (scene_result_id_ == id && published_revision_ >= scene_result_revision_);
        };
        if (!completed_.wait_until(lock, deadline, done)) {
            finish_scene(false, "Harvest operation timed out before a physical result was observed");
        }
        response->success = !shutdown_ && scene_result_id_ == id && scene_success_;
        response->message = shutdown_ ? "Physics backend stopped" : scene_message_;
        scene_service_busy_ = false;
    }
    void finish_scene(bool success, const std::string& message) {
        pending_scene_ = SceneCommand::None;
        scene_result_id_ = scene_id_;
        scene_success_ = success;
        scene_message_ = message;
        scene_result_revision_ = ++revision_;
        completed_.notify_all();
    }
    void reset_metadata() {
        initialized_ = false;
        opening_held_fruit_ = false;
        selected_ = held_ = -1;
        phase_ = "Reset the harvest scene at the observation pose";
        initial_tcp_ = geometry_msgs::msg::PoseStamped{};
        for (auto& fruit : fruits_) {
            fruit.state = PhysicsState::AVAILABLE;
            fruit.left_force = fruit.right_force = 0.;
            fruit.contact_since = fruit.last_contact = fruit.settle_since = -1.;
            fruit.floor_contact = false;
        }
    }
    void apply_scene_command(mjData* data) {
        scene_applied_ = true;
        if (pending_scene_ == SceneCommand::Reset) {
            stop_motion("Scene reset interrupted EEF motion");
            reset_metadata();
            const auto* origin = data->site_xpos + 3 * tip_;
            const auto* rotation = data->site_xmat + 9 * tip_;
            initial_tcp_.header.stamp = node_->now();
            initial_tcp_.header.frame_id = world_frame_;
            initial_tcp_.pose.position.x = origin[0];
            initial_tcp_.pose.position.y = origin[1];
            initial_tcp_.pose.position.z = origin[2];
            mjtNum quaternion[4];
            mju_mat2Quat(quaternion, rotation);
            initial_tcp_.pose.orientation.w = quaternion[0];
            initial_tcp_.pose.orientation.x = quaternion[1];
            initial_tcp_.pose.orientation.y = quaternion[2];
            initial_tcp_.pose.orientation.z = quaternion[3];
            for (std::size_t i = 0; i < fruits_.size(); ++i) {
                const auto& fruit = fruits_[i];
                for (int axis = 0; axis < 3; ++axis) {
                    double value = origin[axis];
                    for (int col = 0; col < 3; ++col) value += rotation[3 * axis + col] * offsets_[3 * i + col];
                    data->qpos[fruit.qpos + axis] = value;
                    data->mocap_pos[3 * fruit.mocap + axis] = value;
                }
                mju_zero(data->qvel + fruit.dof, 6);
                mju_zero(data->qpos + fruit.qpos + 3, 4);
                mju_zero(data->mocap_quat + 4 * fruit.mocap, 4);
                data->qpos[fruit.qpos + 3] = data->mocap_quat[4 * fruit.mocap] = 1.;
                data->eq_active[fruit.weld] = 1;
            }
            mj_forward(model_, data);
            required_snapshot_time_ = data->time + model_->opt.timestep * .5;
            initialized_ = true;
            phase_ = "Fruit stems anchored at the measured observation TCP";
            finish_scene(true, phase_);
            return;
        }
        if (!initialized_) { finish_scene(false, "Reset the harvest scene first"); return; }
        if (pending_scene_ == SceneCommand::Select) {
            if (held_ >= 0) { finish_scene(false, "Release the held fruit before selecting another"); return; }
            if (selected_ < 0) {
                for (std::size_t i = 0; i < fruits_.size(); ++i) {
                    if (fruits_[i].state == PhysicsState::AVAILABLE) { selected_ = static_cast<int>(i); break; }
                }
            }
            phase_ = selected_ >= 0 ? "Approaching fruit " + std::to_string(selected_ + 1) : "All available fruits processed";
            finish_scene(true, phase_);
        } else if (pending_scene_ == SceneCommand::Grasp) {
            if (selected_ < 0 || held_ >= 0 || fruits_[selected_].state != PhysicsState::AVAILABLE) {
                finish_scene(false, "Select an available fruit before grasping"); return;
            }
            if (!stable_contact(selected_, data->time)) {
                finish_scene(false, "Both fingers must establish stable positive contact force on the selected fruit"); return;
            }
            auto& fruit = fruits_[selected_];
            data->eq_active[fruit.weld] = 0;
            fruit.state = PhysicsState::HELD;
            held_ = selected_;
            phase_ = "Carrying fruit " + std::to_string(held_ + 1) + " by finger contact";
            finish_scene(true, phase_);
        } else if (pending_scene_ == SceneCommand::Release) {
            if (held_ < 0) { finish_scene(false, "No fruit is held"); return; }
            if (!fingers_at(open_position_)) { finish_scene(false, "Open both fingers before releasing"); return; }
            release_index_ = held_;
            fruits_[release_index_].state = PhysicsState::RELEASED;
            fruits_[release_index_].settle_since = -1.;
            held_ = -1;
            phase_ = "Waiting for fruit " + std::to_string(release_index_ + 1) + " to settle on the basket floor";
            ++revision_;
        }
    }
    void track_fruits(const mjData* data) {
        const auto* floor_position = data->geom_xpos + 3 * floor_;
        const auto* floor_rotation = data->geom_xmat + 9 * floor_;
        const auto* floor_size = model_->geom_size + 3 * floor_;
        for (std::size_t i = 0; i < fruits_.size(); ++i) {
            auto& fruit = fruits_[i];
            if (fruit.state != PhysicsState::HELD && fruit.state != PhysicsState::RELEASED) continue;
            const auto* position = data->xpos + 3 * fruit.body;
            mjtNum local[3]{}, delta[3];
            mju_sub3(delta, position, floor_position);
            mju_mulMatTVec3(local, floor_rotation, delta);
            mjtNum velocity[6];
            mj_objectVelocity(model_, data, mjOBJ_BODY, fruit.body, velocity, 0);
            const bool inside = std::abs(local[0]) < floor_size[0] - radius_ * .4 &&
                                std::abs(local[1]) < floor_size[1] - radius_ * .4;
            if (fruit.state == PhysicsState::RELEASED && inside && fruit.floor_contact &&
                mju_norm3(velocity + 3) <= settle_speed_) {
                if (fruit.settle_since < 0.) fruit.settle_since = data->time;
                if (data->time - fruit.settle_since >= settle_stable_) {
                    fruit.state = PhysicsState::COLLECTED;
                    selected_ = -1;
                    phase_ = "Fruit " + std::to_string(i + 1) + " settled inside the basket";
                    ++revision_;
                }
            } else {
                fruit.settle_since = -1.;
            }
            bool dropped = local[2] < -radius_ * 2.;
            if (fruit.state == PhysicsState::HELD && !opening_held_fruit_ && data->time - fruit.last_contact > .35) {
                mjtNum from_tip[3];
                mju_sub3(from_tip, position, data->site_xpos + 3 * tip_);
                dropped = dropped || mju_norm3(from_tip) > .10;
            }
            if (dropped) {
                fruit.state = PhysicsState::DROPPED;
                if (held_ == static_cast<int>(i)) held_ = -1;
                phase_ = "Fruit " + std::to_string(i + 1) + " dropped outside the basket";
                ++revision_;
            }
        }
        if (pending_scene_ == SceneCommand::Release && scene_applied_) {
            const auto state = fruits_[release_index_].state;
            if (state == PhysicsState::COLLECTED) finish_scene(true, phase_);
            else if (state == PhysicsState::DROPPED) finish_scene(false, phase_);
        }
    }

    rclcpp::Node::SharedPtr node_;
    const mjModel* model_{nullptr};
    std::vector<Fruit> fruits_;
    std::vector<double> offsets_;
    std::string world_frame_, phase_{"Reset the harvest scene at the observation pose"};
    int tip_{-1}, floor_{-1}, selected_{-1}, held_{-1}, release_index_{-1};
    std::array<int, 2> finger_bodies_{}, finger_qpos_{}, finger_dof_{};
    std::array<double, 2> finger_positions_{}, finger_velocities_{};
    double radius_{.025}, closed_position_{-.008}, open_position_{0.}, position_tolerance_{.0002};
    double velocity_tolerance_{.003}, force_min_{.02}, grasp_stable_{.12}, settle_speed_{.04};
    double settle_stable_{.25}, service_timeout_{12.}, publish_period_{.02}, last_publish_time_{-1.};
    double required_snapshot_time_{-1.}, motion_target_{0.};
    double stop_still_since_{-1.};
    bool initialized_{false}, have_feedback_{false}, shutdown_{false}, scene_applied_{false}, opening_held_fruit_{false};
    bool motion_running_{false}, motion_success_{false}, scene_success_{false};
    bool motion_service_busy_{false}, scene_service_busy_{false};
    bool stopping_{false}, stop_confirmed_{false};
    int motion_grasp_index_{-1};
    std::uint64_t revision_{0}, published_revision_{0}, motion_id_{0}, motion_result_id_{0};
    std::uint64_t scene_id_{0}, scene_result_id_{0}, scene_result_revision_{0};
    std::string motion_message_, scene_message_;
    SceneCommand pending_scene_{SceneCommand::None};
    geometry_msgs::msg::PoseStamped initial_tcp_;
    std::mutex mutex_;
    std::condition_variable completed_;
    rclcpp::CallbackGroup::SharedPtr group_;
    rclcpp::Publisher<PhysicsState>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
    std::vector<rclcpp::Service<Trigger>::SharedPtr> scene_services_;
    rclcpp::Service<CommandEef>::SharedPtr eef_service_;
    rclcpp::Service<Trigger>::SharedPtr stop_service_, ready_service_;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> service_executor_;
    std::thread service_thread_;
};
} // namespace tomato_picker_mujoco

PLUGINLIB_EXPORT_CLASS(tomato_picker_mujoco::HarvestScenePlugin,
    mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
