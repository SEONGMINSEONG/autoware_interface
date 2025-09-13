#include "autoware_interface/autoware_interface.hpp"

namespace autoware_interface {

using namespace std::chrono_literals;

AutowareInterface::AutowareInterface() : Node("autoware_interface") 
{
    /* From Autoware */
    AW_cmd_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
        "/control/command/control_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::aw_cmd_callback, this, std::placeholders::_1));
    AW_gear_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
        "/control/command/gear_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::aw_gear_cmd_callback, this, std::placeholders::_1));

    /* From vehicle_interface */
    velocity_status_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/can_message_receiver/velocity_status", rclcpp::QoS(1), std::bind(&AutowareInterface::velocity_status_callback, this, std::placeholders::_1));
    steer_status_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/can_message_receiver/steer_status", rclcpp::QoS(1), std::bind(&AutowareInterface::steer_status_callback, this, std::placeholders::_1));
    gear_status_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/can_message_receiver/gear_status", rclcpp::QoS(1), std::bind(&AutowareInterface::gear_status_callback, this, std::placeholders::_1));
    vehicle_mode_status_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/can_message_receiver/vehicle_mode_status", rclcpp::QoS(1), std::bind(&AutowareInterface::vehicle_mode_status_callback, this, std::placeholders::_1));

    /* TO vehicle_interface */
    gear_cmd_pub_ = this->create_publisher<std_msgs::msg::Int8>(
        "/can_message_sender/gear_cmd", rclcpp::QoS(1));

    /* To Autoware */
    AW_velocity_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", rclcpp::QoS(1));
    AW_steer_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", rclcpp::QoS(1));
    AW_gear_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::GearReport>(
        "/vehicle/status/gear_status", rclcpp::QoS(1));
    AW_control_mode_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", rclcpp::QoS(1));
  
    /* To TwistController */
    TC_velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/twist_controller/input/velocity_cmd", rclcpp::QoS(1));
    TC_steer_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/twist_controller/input/steer_cmd", rclcpp::QoS(1));
    TC_gear_matched_pub_ = this->create_publisher<std_msgs::msg::Int8>(
        "/twist_controller/input/gear_matched", rclcpp::QoS(1));

    // Timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&AutowareInterface::timer_callback, this));
}

void AutowareInterface::aw_cmd_callback(const autoware_control_msgs::msg::Control::SharedPtr msg) 
{
    velocity_cmd_ = msg->longitudinal.velocity;
    steer_cmd_ = msg->lateral.steering_tire_angle;
}

void AutowareInterface::aw_gear_cmd_callback(const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg) 
{
    gear_mode_ = msg->command;
    if (gear_mode_ == autoware_vehicle_msgs::msg::GearCommand::DRIVE)
    {
        gear_cmd_ = 0;
    }
    else if(gear_mode_ == autoware_vehicle_msgs::msg::GearCommand::REVERSE)
    {
        gear_cmd_ = 1;
    }

    std_msgs::msg::Int8 gear_cmd_msg;
    gear_cmd_msg.data = gear_cmd_;
    gear_cmd_pub_->publish(gear_cmd_msg);
}

void AutowareInterface::velocity_status_callback(const std_msgs::msg::Float64::SharedPtr msg) 
{
    velocity_status_ = msg->data;
}

void AutowareInterface::steer_status_callback(const std_msgs::msg::Float64::SharedPtr msg) 
{
    steer_status_ = msg->data;
}

void AutowareInterface::gear_status_callback(const std_msgs::msg::Int8::SharedPtr msg) 
{
    gear_status_ = msg->data;
}

void AutowareInterface::vehicle_mode_status_callback(const std_msgs::msg::Int8::SharedPtr msg) 
{
    vehicle_status_ = msg->data;
}


void AutowareInterface::timer_callback() {
    // To Autoware
    auto now = this->now();

    autoware_vehicle_msgs::msg::VelocityReport velocity_report_msg;
    velocity_report_msg.header.stamp = now;
    velocity_report_msg.header.frame_id = "base_link";
    velocity_report_msg.longitudinal_velocity = velocity_status_;
    AW_velocity_pub_->publish(velocity_report_msg);

    autoware_vehicle_msgs::msg::SteeringReport steering_report_msg;
    steering_report_msg.stamp = now;
    steering_report_msg.steering_tire_angle = steer_status_;
    AW_steer_pub_->publish(steering_report_msg);

    autoware_vehicle_msgs::msg::ControlModeReport control_mode_msg;
    control_mode_msg.stamp = now;
    control_mode_msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    AW_control_mode_pub_->publish(control_mode_msg);

    autoware_vehicle_msgs::msg::GearReport gear_report_msg;
    gear_report_msg.stamp = now;
    // 0:D, 1:R  
    switch (gear_status_) {
        case 0: // D
            gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::DRIVE;
            break;
        case 1: // R
            gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::REVERSE;
            break;
        default:
            gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::NONE;
            break;
    }
    // AW_gear_pub_->publish(gear_report_msg);

    // To TwistController
    std_msgs::msg::Float64 velocity_cmd_msg;
    velocity_cmd_msg.data = velocity_cmd_;
    TC_velocity_cmd_pub_->publish(velocity_cmd_msg);

    std_msgs::msg::Float64 steer_cmd_msg;
    steer_cmd_msg.data = steer_cmd_;
    TC_steer_cmd_pub_->publish(steer_cmd_msg);

    std_msgs::msg::Int8 gear_matched_msg;
    gear_matched_msg.data = (gear_cmd_ == gear_status_) ? true : false;
    TC_gear_matched_pub_->publish(gear_matched_msg);
}

} // namespace autoware_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autoware_interface::AutowareInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
