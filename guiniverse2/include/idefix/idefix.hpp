#pragma once

#include "geometry_msgs/msg/twist_stamped.hpp"
#include <guiniverse2/imgui_utils.hpp>
#include <guiniverse2/guiniverse2.hpp>
#include <guiniverse2/joystick_input.hpp>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>


class Idefix : public RobotController
{
public:
    Idefix();
    ~Idefix();

    void on_gui_frame(GLFWwindow* window) override;
private:

    JoystickInput joystick_input;

    rclcpp::Node::SharedPtr node;

    // Driving

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TwistPublisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_TwistStampedPublisher;
    geometry_msgs::msg::TwistStamped m_TwistStampedMessage;

    struct {
        float scalar = 0.05f;

        bool stamped = true;
        bool gas_button = false;
        bool publish_cmd = true;
        bool enable_publish_on_gas = true;

    } m_Input;

    // Arm control
};