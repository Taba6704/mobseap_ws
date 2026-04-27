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
  this->declare_parameter("search_yaw_rate", -0.1);          // Right twist, adjust sign if needed
  this->declare_parameter("brake_yaw_rate", 0.20);           // BRAKE thrust, opposite of search_yaw_rate
  this->declare_parameter("brake_time_sec", 0.6);            // Counter-steer time (seconds)
  this->declare_parameter("yaw_kp", 0.25);                   // Yaw P Controller gain
  this->declare_parameter("max_yaw_rate", 0.40);             // Max angular (z) /cmd_vel_auto message while in APPROACH state (mapped from -1 to 1)
  this->declare_parameter("approach_speed", 0.35);           // Max linear (x) /cmd_vel_auto message while in APPROACH state (mapped from -1 to 1)
  this->declare_parameter("center_tolerance", 0.20);         //
  this->declare_parameter("lock_time_sec", 0.50);            //
  this->declare_parameter("target_timeout_sec", 0.75);       //
  this->declare_parameter("hold_target_distance_m", 2.0);    // When target reaches 2.0 meters -> enter HOLD State
  this->declare_parameter("hold_inner_distance_m", 1.5);     // Closest USV will be allowed from target before emergency back thrust
  this->declare_parameter("hold_outer_distance_m", 2.8);     // Farthest USV will be allowed from target before entering APPROACH state
  this->declare_parameter("hold_distance_deadband_m", 0.25); // Max allowed error for target distance compared to hold distance
  this->declare_parameter("hold_distance_kp", 0.20);         // HOLD state P controller gain
  this->declare_parameter("max_hold_speed", 0.15);           // Max /cmd_vel_auto message while in HOLD state (mapped from -1 to 1)

  this->get_parameter("search_yaw_rate", search_yaw_rate_);
  this->get_parameter("brake_yaw_rate", brake_yaw_rate_);
  this->get_parameter("brake_time_sec", brake_time_sec_);
  this->get_parameter("yaw_kp", yaw_kp_);
  this->get_parameter("max_yaw_rate", max_yaw_rate_);
  this->get_parameter("approach_speed", approach_speed_);
  this->get_parameter("center_tolerance", center_tolerance_);
  this->get_parameter("lock_time_sec", lock_time_sec_);
  this->get_parameter("target_timeout_sec", target_timeout_sec_);
  this->get_parameter("hold_target_distance_m", hold_target_distance_m_);
  this->get_parameter("hold_inner_distance_m", hold_inner_distance_m_);
  this->get_parameter("hold_outer_distance_m", hold_outer_distance_m_);
  this->get_parameter("hold_distance_deadband_m", hold_distance_deadband_m_);
  this->get_parameter("hold_distance_kp", hold_distance_kp_);
  this->get_parameter("max_hold_speed", max_hold_speed_);

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
  brake_start_time_ = this->now();

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
        state_ = PlannerState::BRAKE;
        brake_start_time_ = now;
        lock_timer_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Target detected. Switching to BRAKE.");
      } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = search_yaw_rate_;
      }
      break;
    }

    case PlannerState::BRAKE:
    {
      if (!target_recent) {
        state_ = PlannerState::SEARCH;
        lock_timer_active_ = false;
        RCLCPP_WARN(this->get_logger(), "Target lost during BRAKE. Returning to SEARCH.");
        break;
      }

      cmd.linear.x = 0.0;
      cmd.angular.z = brake_yaw_rate_;

      if ((now - brake_start_time_).seconds() >= brake_time_sec_) {
        state_ = PlannerState::ALIGN;
        lock_timer_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Brake complete. Switching to ALIGN.");
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

      // Once inside the hold band, switch to HOLD
      if (target_range_m_ <= hold_outer_distance_m_ && target_range_m_ >= hold_inner_distance_m_) {
        state_ = PlannerState::HOLD;
        lock_timer_active_ = false;
        RCLCPP_INFO(
          this->get_logger(),
          "Target inside hold band %.2f m to %.2f m. Switching to HOLD.",
          hold_inner_distance_m_,
          hold_outer_distance_m_);
        break;
      }

      double yaw_cmd = -yaw_kp_ * pixel_error_;
      yaw_cmd = clamp(yaw_cmd, -max_yaw_rate_, max_yaw_rate_);

      // Reduce forward speed if target is off-center
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

    case PlannerState::HOLD:
    {
      if (!target_recent) {
        state_ = PlannerState::SEARCH;
        lock_timer_active_ = false;
        RCLCPP_WARN(this->get_logger(), "Target lost during HOLD. Returning to SEARCH.");
        break;
      }

      // If target gets too far away, go approach again.
      if (target_range_m_ > hold_outer_distance_m_) {
        state_ = PlannerState::APPROACH;
        lock_timer_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Target beyond hold range. Returning to APPROACH.");
        break;
      }

      double yaw_cmd = -yaw_kp_ * pixel_error_;
      yaw_cmd = clamp(yaw_cmd, -max_yaw_rate_, max_yaw_rate_);

      // Positive error means target is farther than desired.
      // Negative error means target is too close.
      double distance_error = target_range_m_ - hold_target_distance_m_;

      double hold_speed_cmd = 0.0;

      if (std::abs(distance_error) > hold_distance_deadband_m_) {
        hold_speed_cmd = hold_distance_kp_ * distance_error;
        hold_speed_cmd = clamp(hold_speed_cmd, -max_hold_speed_, max_hold_speed_);
      }

      // Safety: if closer than inner distance, allow reverse correction.
      // If reverse is too aggressive in testing, lower max_hold_speed.
      if (target_range_m_ < hold_inner_distance_m_) {
        hold_speed_cmd = -max_hold_speed_;
      }

      cmd.linear.x = hold_speed_cmd;
      cmd.angular.z = yaw_cmd;

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