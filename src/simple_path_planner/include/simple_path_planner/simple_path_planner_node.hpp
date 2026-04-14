#ifndef SIMPLE_PATH_PLANNER__SIMPLE_PATH_PLANNER_NODE_HPP_
#define SIMPLE_PATH_PLANNER__SIMPLE_PATH_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

class SimplePathPlannerNode : public rclcpp::Node
{
public:
  SimplePathPlannerNode();

private:
  enum class PlannerState
  {
    SEARCH,
    ALIGN,
    APPROACH,
    COMPLETE
  };

  void targetVisibleCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void pixelErrorCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void targetRangeCallback(const std_msgs::msg::Float64::SharedPtr msg);

  void controlLoop();
  void publishStop();
  double clamp(double value, double min_val, double max_val);

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_visible_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pixel_error_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_range_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_auto_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Current target data
  bool target_visible_;
  double pixel_error_;
  double target_range_m_;
  rclcpp::Time last_target_seen_time_;
  rclcpp::Time lock_start_time_;

  // State
  PlannerState state_;
  bool lock_timer_active_;

  // Parameters
  double search_yaw_rate_;
  double yaw_kp_;
  double max_yaw_rate_;
  double approach_speed_;
  double center_tolerance_;
  double lock_time_sec_;
  double stop_distance_m_;
  double target_timeout_sec_;
};

#endif  // SIMPLE_PATH_PLANNER__SIMPLE_PATH_PLANNER_NODE_HPP_
