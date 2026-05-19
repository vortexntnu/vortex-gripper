#include "gripper_reference_filter/gripper_reference_filter_ros.hpp"
#include <spdlog/spdlog.h>
#include <mutex>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include "gripper_reference_filter/gripper_reference_filter_ros_utils.hpp"

const auto start_message = R"(

  ____       _                         ____       __                                _____ _ _ _
 / ___|_ __ (_)_ __  _ __   ___ _ __  |  _ \ ___ / _| ___ _ __ ___ _ __   ___ ___  |  ___(_) | |_ ___ _ __
| |  _| '_ \| | '_ \| '_ \ / _ \ '__| | |_) / _ \ |_ / _ \ '__/ _ \ '_ \ / __/ _ \ | |_  | | | __/ _ \ '__|
| |_| | |_) | | |_) | |_) |  __/ |    |  _ <  __/  _|  __/ | |  __/ | | | (_|  __/ |  _| | | | ||  __/ |
 \____| .__/|_| .__/| .__/ \___|_|    |_| \_\___|_|  \___|_|  \___|_| |_|\___\___| |_|   |_|_|\__\\\___|_|
      |_|     |_|   |_|

 )";

namespace vortex::guidance {

GripperReferenceFilterNode::GripperReferenceFilterNode(const rclcpp::NodeOptions& options)
    : Node("gripper_reference_filter_node", options) {
    set_subscribers_and_publisher();

    set_action_server();

    set_refererence_filter();

    held_reference_republish_timer_ = this->create_wall_timer(
        time_step_ms_,
        [this]() { republish_held_reference_tick(); });

    spdlog::info(start_message);
}

GripperReferenceFilterNode::~GripperReferenceFilterNode() {
    preempted_ = true;
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
}

void GripperReferenceFilterNode::set_subscribers_and_publisher() {
    const std::string guidance_topic =
        this->declare_parameter<std::string>("topics.guidance_gripper");
    const std::string gripper_state_topic =
        this->declare_parameter<std::string>("topics.gripper_state");

    const auto qos_sensor_data = vortex::utils::qos_profiles::sensor_data_profile(1);

    reference_pub_ = this->create_publisher<vortex_msgs::msg::GripperReferenceFilter>(
        guidance_topic, qos_sensor_data);

    reference_sub_ = this->create_subscription<vortex_msgs::msg::GripperState>(
        gripper_state_topic, qos_sensor_data,
        [this](const vortex_msgs::msg::GripperState::SharedPtr state_msg) {
            reference_callback(state_msg);
        });
}

void GripperReferenceFilterNode::set_action_server() {
    this->declare_parameter<std::string>("action_servers.gripper_reference_filter");
    const std::string action_server_name =
        this->get_parameter("action_servers.gripper_reference_filter").as_string();

    action_server_ = rclcpp_action::create_server<
        vortex_msgs::action::GripperReferenceFilterWaypoint>(
        this, action_server_name,
        [this](const auto& uuid, auto goal) {
            return handle_goal(uuid, std::move(goal));
        },
        [this](auto goal_handle) { return handle_cancel(goal_handle); },
        [this](auto goal_handle) { handle_accepted(goal_handle); });
}

void GripperReferenceFilterNode::set_refererence_filter() {
    const int time_step_ms_param =
        this->declare_parameter<int>("time_step_ms", 10);
    time_step_ms_ = std::chrono::milliseconds(time_step_ms_param);

    this->declare_parameter<std::vector<double>>("zeta");
    this->declare_parameter<std::vector<double>>("omega");

    const std::vector<double> zeta = this->get_parameter("zeta").as_double_array();
    const std::vector<double> omega = this->get_parameter("omega").as_double_array();

    const Eigen::Vector2d zeta_eigen = Eigen::Map<const Eigen::Vector2d>(zeta.data());
    const Eigen::Vector2d omega_eigen = Eigen::Map<const Eigen::Vector2d>(omega.data());

    const GripperReferenceFilterParams filter_params{omega_eigen, zeta_eigen};
    gripper_reference_filter_ = std::make_unique<GripperReferenceFilter>(filter_params);
}

void GripperReferenceFilterNode::reference_callback(
    const vortex_msgs::msg::GripperState::SharedPtr state_msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    measured_reference_ << state_msg->roll, state_msg->pinch;
}

rclcpp_action::GoalResponse GripperReferenceFilterNode::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const vortex_msgs::action::GripperReferenceFilterWaypoint::Goal>
    /*goal*/) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        holding_reference_ = false;
    }
    spdlog::info("Accepted goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperReferenceFilterNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GripperReferenceFilterWaypoint>> /*goal_handle*/) {
    spdlog::info("Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperReferenceFilterNode::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GripperReferenceFilterWaypoint>> goal_handle) {
    std::lock_guard<std::mutex> lock(execute_mutex_);
    preempted_ = true;
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
    preempted_ = false;

    execute_thread_ =
        std::thread([this, goal_handle]() { execute(goal_handle); });
}

void GripperReferenceFilterNode::latch_current_state_as_held_reference() {
    if (!reference_pub_) {
        return;
    }
    auto held_message = std::make_unique<vortex_msgs::msg::GripperReferenceFilter>(
        fill_reference_msg(gripper_reference_filter_->reference_output()));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_published_reference_ = *held_message;
        holding_reference_ = true;
    }

    reference_pub_->publish(std::move(held_message));
}

void GripperReferenceFilterNode::republish_held_reference_tick() {
    auto held_message = std::make_unique<vortex_msgs::msg::GripperReferenceFilter>();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!holding_reference_ || !reference_pub_) {
            return;
        }
        *held_message = last_published_reference_;
    }

    reference_pub_->publish(std::move(held_message));
}

void GripperReferenceFilterNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GripperReferenceFilterWaypoint>> goal_handle) {
    spdlog::info("Executing goal");

    const Eigen::Vector2d filter_seed = [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        if (holding_reference_) {
            Eigen::Vector2d seed;
            seed << last_published_reference_.roll,
                    last_published_reference_.pinch;
            return seed;
        }
        return measured_reference_;
    }();
    gripper_reference_filter_->reset(filter_seed);

    const vortex_msgs::msg::GripperWaypoint waypoint_goal =
        goal_handle->get_goal()->waypoint;

    const uint8_t mode = waypoint_goal.mode;

    double convergence_threshold = goal_handle->get_goal()->convergence_threshold;
    if (convergence_threshold <= 0.0) {
        convergence_threshold = 0.1;
        spdlog::warn(
            "GripperReferenceFilter: Invalid convergence_threshold received (<= 0). "
            "Using default 0.1");
    }

    const Eigen::Vector2d goal_reference =
        apply_mode_logic(fill_reference_goal(waypoint_goal), filter_seed, mode);

    const double time_step_seconds = time_step_ms_.count() / 1000.0;

    auto result = std::make_shared<
        vortex_msgs::action::GripperReferenceFilterWaypoint::Result>();

    rclcpp::Rate loop_rate(1000.0 / time_step_ms_.count());

    while (rclcpp::ok()) {
        if (preempted_.load()) {
            latch_current_state_as_held_reference();
            result->success = false;
            goal_handle->abort(result);
            spdlog::info("Goal preempted by newer goal");
            return;
        }

        if (goal_handle->is_canceling()) {
            latch_current_state_as_held_reference();
            result->success = false;
            goal_handle->canceled(result);
            spdlog::info("Goal canceled");
            return;
        }

        gripper_reference_filter_->step(goal_reference, time_step_seconds);

        const Eigen::Vector2d filter_output = gripper_reference_filter_->reference_output();

        auto reference_message =
            std::make_unique<vortex_msgs::msg::GripperReferenceFilter>(
                fill_reference_msg(filter_output));
        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_published_reference_ = *reference_message;
        }
        reference_pub_->publish(std::move(reference_message));

        const Eigen::Vector2d convergence_error =
            compute_convergence_error(filter_output, goal_reference, mode);

        if (convergence_error.norm() < convergence_threshold) {
            gripper_reference_filter_->snap_to(goal_reference);
            auto final_message =
                std::make_unique<vortex_msgs::msg::GripperReferenceFilter>(
                    fill_reference_msg(gripper_reference_filter_->reference_output()));
            {
                std::lock_guard<std::mutex> lock(mutex_);
                last_published_reference_ = *final_message;
                holding_reference_ = true;
            }
            reference_pub_->publish(std::move(final_message));
            result->success = true;
            goal_handle->succeed(result);
            spdlog::info("Goal reached");
            return;
        }

        loop_rate.sleep();
    }

    if (!rclcpp::ok() && goal_handle->is_active()) {
        result->success = false;
        try {
            goal_handle->abort(result);
        } catch (...) {
            // Ignore exceptions during shutdown
        }
    }
}

}  // namespace vortex::guidance

RCLCPP_COMPONENTS_REGISTER_NODE(vortex::guidance::GripperReferenceFilterNode)
