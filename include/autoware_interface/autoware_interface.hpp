#ifndef AUTOWARE_INTERFACE_HPP_
#define AUTOWARE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include <chrono>

namespace autoware_interface {

class AutowareInterface : public rclcpp::Node {
public:
    AutowareInterface();

private:
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr AW_cmd_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr AW_gear_cmd_sub_;
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr gear_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr vehicle_mode_status_sub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr gear_cmd_pub_;

    
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr AW_velocity_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr AW_steer_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr AW_gear_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr AW_control_mode_pub_;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_velocity_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_steer_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr TC_gear_matched_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr vehicle_status_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    double velocity_cmd_ = 0.0;
    double velocity_status_ = 0.0;
    double steer_cmd_ = 0.0;
    double steer_status_ = 0.0;
    int32_t gear_cmd_ = 0;
    int32_t gear_mode_ = 0;
    int8_t gear_status_ = 0;
    int8_t vehicle_status_ = 0;


    void aw_cmd_callback(const autoware_control_msgs::msg::Control::SharedPtr msg);
    void aw_gear_cmd_callback(const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg);
    void velocity_status_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void steer_status_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void gear_status_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void vehicle_mode_status_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void timer_callback();
};

} // namespace autoware_interface

#endif // AUTOWARE_INTERFACE_HPP_
