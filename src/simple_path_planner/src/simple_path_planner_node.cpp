#include "simple_path_planner/simple_path_planner_node.hpp"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

SimplePathPlannerNode::SimplePathPlannerNode()
: Node("simple_path_planner_node"),
  target_visible_(false),
  pixel_error_(0.0),
  target_range_m_(999.0),
  state_(PlannerState::SEARCH),
  lock_timer_active_(false)
{
  // Parameters
  this->declare_parameter("search_yaw_rate", -0.30);      // clockwise, adjust sign if needed
  this->declare_parameter("yaw_kp", 0.80);
  this->declare_parameter("max_yaw_rate", 0.60);
  this->declare_parameter("approach_speed", 0.35);
  this->declare_parameter("center_tolerance", 0.08);
  this->declare_parameter("lock_time_sec", 0.50);
  this->declare_parameter("stop_distance_m", 2.0);
  this->declare_parameter("target_timeout_sec", 0.75);

  this->get_parameter("search_yaw_rate", search_yaw_rate_);
  this->get_parameter("yaw_kp", yaw_kp_);
  this->get_parameter("max_yaw_rate", max_yaw_rate_);
  this->get_parameter("approach_speed", approach_speed_);
  this->get_parameter("center_tolerance", center_tolerance_);
  this->get_parameter("lock_time_sec", lock_time_sec_);
  this->get_parameter("stop_distance_m", stop_distance_m_);
  this->get_parameter("target_timeout_sec", target_timeout_sec_);

  // Subscribers
  target_visible_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/target_visible", 10,
    std::bind(&SimplePathPlannerNode::targetVisibleCallback, this, std::placeholders::_1));

  pixel_error_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/target_pixel_error", 10,
    std::bind(&SimplePathPlannerNode::pixelErrorCallback, this, std::placeholders::_1));

  target_range_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/target_range_m", 10,
    std::bind(&SimplePathPlannerNode::targetRangeCallback, this, std::placeholders::_1));

  // Publisher
  cmd_vel_auto_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_auto", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
    100ms, std::bind(&SimplePathPlannerNode::controlLoop, this));

  last_target_seen_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Simple path planner node started.");
}

void SimplePathPlannerNode::targetVisibleCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  target_visible_ = msg->data;
  if (target_visible_) {
    last_target_seen_time_ = this->now();
  }
}

void SimplePathPlannerNode::pixelErrorCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  pixel_error_ = msg->data;
}

void SimplePathPlannerNode::targetRangeCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  target_range_m_ = msg->data;
}

double SimplePathPlannerNode::clamp(double value, double min_val, double max_val)
{
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

void SimplePathPlannerNode::publishStop()
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;
  cmd_vel_auto_pub_->publish(cmd);
}

void SimplePathPlannerNode::controlLoop()
{
  geometry_msgs::msg::Twist cmd;
  rclcpp::Time now = this->now();

  // Consider target lost if not seen recently
  bool target_recent =
    target_visible_ &&
    ((now - last_target_seen_time_).seconds() <= target_timeout_sec_);

  switch (state_) {
    case PlannerState::SEARCH:
    {
      if (target_recent) {
        state_ = PlannerState::ALIGN;
        lock_timer_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Target detected. Switching to ALIGN.");
      } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = search_yaw_rate_;
      }
      break;
    }

    case PlannerState::ALIGN:
    {
      if (!target_recent) {
        state_ = PlannerState::SEARCH;
        lock_timer_active_ = false;
        RCLCPP_WARN(this->get_logger(), "Target lost. Returning to SEARCH.");
        break;
      }

      double yaw_cmd = -yaw_kp_ * pixel_error_;
      yaw_cmd = clamp(yaw_cmd, -max_yaw_rate_, max_yaw_rate_);

      cmd.linear.x = 0.0;
      cmd.angular.z = yaw_cmd;

      if (std::abs(pixel_error_) < center_tolerance_) {
        if (!lock_timer_active_) {
          lock_start_time_ = now;
          lock_timer_active_ = true;
        } else if ((now - lock_start_time_).seconds() >= lock_time_sec_) {
          state_ = PlannerState::APPROACH;
          lock_timer_active_ = false;
          RCLCPP_INFO(this->get_logger(), "Target centered and stable. Switching to APPROACH.");
        }
      } else {
        lock_timer_active_ = false;
      }
      break;
    }

    case PlannerState::APPROACH:
    {
      if (!target_recent) {
        state_ = PlannerState::SEARCH;
        lock_timer_active_ = false;
        RCLCPP_WARN(this->get_logger(), "Target lost during approach. Returning to SEARCH.");
        break;
      }

      if (target_range_m_ <= stop_distance_m_) {
        state_ = PlannerState::COMPLETE;
        RCLCPP_INFO(this->get_logger(), "Target reached within %.2f m. Mission complete.", stop_distance_m_);
        break;
      }

      double yaw_cmd = -yaw_kp_ * pixel_error_;
      yaw_cmd = clamp(yaw_cmd, -max_yaw_rate_, max_yaw_rate_);

      // Optional: reduce forward speed if target is off-center
      double forward_scale = 1.0;
      if (std::abs(pixel_error_) > center_tolerance_) {
        forward_scale = 0.5;
      }
      if (std::abs(pixel_error_) > 0.20) {
        forward_scale = 0.2;
      }

      cmd.linear.x = approach_speed_ * forward_scale;
      cmd.angular.z = yaw_cmd;
      break;
    }

    case PlannerState::COMPLETE:
    {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      break;
    }
  }

  cmd_vel_auto_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
