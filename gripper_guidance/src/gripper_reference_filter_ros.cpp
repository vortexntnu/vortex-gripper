#include "gripper_reference_filter/gripper_reference_filter_ros.hpp"
#include <spdlog/spdlog.h>
#include <mutex>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/math.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/types.hpp>

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
    time_step_ = std::chrono::milliseconds(10);

    set_subscribers_and_publisher();

    set_action_server();

    set_refererence_filter();

    spdlog::info(start_message);
}

void GripperReferenceFilterNode::set_subscribers_and_publisher() {
    std::string guidance_topic = 
        this->declare_parameter<std::string>("topics.guidance_gripper");
    std::string gripper_state_topic = 
        this->declare_parameter<std::string>("topics.gripper_state");

    auto qos_sensor_data = vortex::utils::qos_profiles::sensor_data_profile(1);
    
    reference_pub_ = this->create_publisher<vortex_msgs::msg::GripperReferenceFilter>(
        guidance_topic, qos_sensor_data);

    reference_sub_ = this->create_subscription<vortex_msgs::msg::GripperState>(
        gripper_state_topic, qos_sensor_data,
        std::bind(&GripperReferenceFilterNode::reference_callback, this,
                  std::placeholders::_1));
}

void GripperReferenceFilterNode::set_action_server() {
    this->declare_parameter<std::string>("action_servers.gripper_reference_filter");
    std::string action_server_name =
        this->get_parameter("action_servers.gripper_reference_filter").as_string();
    cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    action_server_ = rclcpp_action::create_server<
        vortex_msgs::action::GripperReferenceFilterWaypoint>(
        
        this, 

        action_server_name,
        
        std::bind(&GripperReferenceFilterNode::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        
        std::bind(&GripperReferenceFilterNode::handle_cancel, this,
                  std::placeholders::_1),
        
        std::bind(&GripperReferenceFilterNode::handle_accepted, this,
                  std::placeholders::_1),

        rcl_action_server_get_default_options(), cb_group_);
}

void GripperReferenceFilterNode::set_refererence_filter() {
    this->declare_parameter<std::vector<double>>("zeta");
    this->declare_parameter<std::vector<double>>("omega");

    std::vector<double> zeta = this->get_parameter("zeta").as_double_array();
    std::vector<double> omega = this->get_parameter("omega").as_double_array();

    Eigen::Vector2d zeta_eigen = Eigen::Map<Eigen::Vector2d>(zeta.data());
    Eigen::Vector2d omega_eigen = Eigen::Map<Eigen::Vector2d>(omega.data());

    GripperReferenceFilterParams filter_params{omega_eigen, zeta_eigen};
    gripper_reference_filter_ = std::make_unique<GripperReferenceFilter>(filter_params);
}

void GripperReferenceFilterNode::reference_callback(
    const vortex_msgs::msg::GripperState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    reference_ << msg->roll, msg->pinch;
}

rclcpp_action::GoalResponse GripperReferenceFilterNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const vortex_msgs::action::GripperReferenceFilterWaypoint::Goal>
        goal) {
    (void)uuid;
    (void)goal;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (goal_handle_) {
            if (goal_handle_->is_active()) {
                spdlog::info("Aborting current goal and accepting new goal");
                preempted_goal_id_ = goal_handle_->get_goal_id();
            }
        }
    }
    spdlog::info("Accepted goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperReferenceFilterNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GripperReferenceFilterWaypoint>> goal_handle) {
    spdlog::info("Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperReferenceFilterNode::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GripperReferenceFilterWaypoint>> goal_handle) {
    std::thread{std::bind(&GripperReferenceFilterNode::execute, this,
                std::placeholders::_1), goal_handle}.detach();
}

Eigen::Vector6d GripperReferenceFilterNode::fill_reference_state() {
    Eigen::Vector6d x = Eigen::Vector6d::Zero();
   
    x(0) = reference_(0);                           // roll
    x(1) = reference_(1);                           // pinch
    x(2) = 0.0;                                     // roll_dot
    x(3) = 0.0;                                     // pinch_dot
    x(4) = 0.0;                                     // roll_dotdot
    x(5) = 0.0;                                     // pinch_dotdot

    return x;
}

Eigen::Vector2d GripperReferenceFilterNode::fill_reference_goal(
    const vortex_msgs::msg::GripperWaypoint& goal) { 

    double roll{goal.roll.roll};
    double pinch{goal.pinch.pinch};

    Eigen::Vector2d reference;
    reference << roll, pinch;

    return reference;
}

vortex_msgs::msg::GripperReferenceFilter GripperReferenceFilterNode::fill_reference_msg() {
    vortex_msgs::msg::GripperReferenceFilter feedback_msg;
    
    feedback_msg.roll        = x_(0); 
    feedback_msg.pinch       = x_(1);
    feedback_msg.roll_dot    = x_(2);
    feedback_msg.pinch_dot   = x_(3);
    feedback_msg.roll_dotdot  = x_(4);
    feedback_msg.pinch_dotdot = x_(5);
    
    return feedback_msg;
}

Eigen::Vector2d GripperReferenceFilterNode::apply_mode_logic(
    const Eigen::Vector2d& reference_in, uint8_t mode) {

    Eigen::Vector2d reference_out = reference_in;

    switch (mode) {
        case vortex_msgs::msg::GripperWaypoint::ROLL_AND_PINCH:
            break;

        case vortex_msgs::msg::GripperWaypoint::ONLY_ROLL:
            reference_out(1) = reference_(1);
            break;

        case vortex_msgs::msg::GripperWaypoint::ONLY_PINCH:
            reference_out(0) = reference_(0);
            break;
    }

    return reference_out;
}

void GripperReferenceFilterNode::publish_hold_reference() {
    if (!reference_pub_) {
        return;
    }
    const double roll  = reference_(0);
    const double pinch = reference_(1);

    vortex_msgs::msg::GripperReferenceFilter hold_msg;
    hold_msg.roll         = roll;
    hold_msg.pinch        = pinch;
    hold_msg.roll_dot     = 0.0;
    hold_msg.pinch_dot    = 0.0;
    hold_msg.roll_dotdot  = 0.0;
    hold_msg.pinch_dotdot = 0.0;

    reference_pub_->publish(hold_msg);
}

void GripperReferenceFilterNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GripperReferenceFilterWaypoint>> goal_handle) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }

    spdlog::info("Executing goal");

    x_ = fill_reference_state();

    const vortex_msgs::msg::GripperWaypoint goal =
        goal_handle->get_goal()->waypoint;

    uint8_t mode = goal.mode;

    double convergence_threshold = goal_handle->get_goal()->convergence_threshold;
    if (convergence_threshold <= 0.0) {
        convergence_threshold = 0.1;
        spdlog::warn(
            "GripperReferenceFilter: Invalid convergence_threshold received (<= 0). "
            "Using default 0.1");
    }

    Eigen::Vector2d reference_temp = fill_reference_goal(goal);
    Eigen::Vector2d goal_reference = apply_mode_logic(reference_temp, mode); // ← fixed: local, not reference_

    auto feedback = std::make_shared<
        vortex_msgs::action::GripperReferenceFilterWaypoint::Feedback>();
    auto result = std::make_shared<
        vortex_msgs::action::GripperReferenceFilterWaypoint::Result>();

    rclcpp::Rate loop_rate(1000.0 / time_step_.count());

    while (rclcpp::ok()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle->get_goal_id() == preempted_goal_id_) {
                publish_hold_reference();
                result->success = false;
                goal_handle->abort(result);
                return;
            }
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle->is_canceling()) {
                publish_hold_reference();
                result->success = false;
                goal_handle->canceled(result);
                spdlog::info("Goal canceled");
                return;
            }
        }

        Eigen::Vector6d x_dot = gripper_reference_filter_->calculate_x_dot(x_, goal_reference); // ← fixed
        x_ += x_dot * time_step_.count() / 1000.0;

        vortex_msgs::msg::GripperReferenceFilter feedback_msg = fill_reference_msg();
        feedback->reference = feedback_msg;
        reference_pub_->publish(feedback_msg);
        goal_handle->publish_feedback(feedback);

        if ((x_.head(2) - goal_reference).norm() < convergence_threshold) { // ← fixed
            result->success = true;
            goal_handle->succeed(result);
            x_.head(2) = goal_reference; // ← fixed
            vortex_msgs::msg::GripperReferenceFilter final_msg = fill_reference_msg();
            reference_pub_->publish(final_msg);
            spdlog::info("Goal reached");
            return;
        }

        loop_rate.sleep();
    }

    if (!rclcpp::ok() && goal_handle->is_active()) {
        auto result = std::make_shared<
            vortex_msgs::action::GripperReferenceFilterWaypoint::Result>();
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
