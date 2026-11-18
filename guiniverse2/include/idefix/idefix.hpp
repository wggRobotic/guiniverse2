#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <guiniverse2/imgui_utils.hpp>
#include <guiniverse2/guiniverse2.hpp>
#include <guiniverse2/joystick_input.hpp>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>


class Idefix : public RobotController
{
public:
    Idefix();
    ~Idefix();

    void on_gui_frame(GLFWwindow* window) override;
private:

    JoystickInput joystick_input;

    rclcpp::Node::SharedPtr node;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TwistPublisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_TwistStampedPublisher;
    geometry_msgs::msg::TwistStamped m_TwistStampedMessage;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_GaitModePublisher;
    std_msgs::msg::String m_GaitModeMessage;
    uint32_t gait_index = 0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_GaitDurationPublisher;
    std_msgs::msg::Float64 m_GaitDurationMessage;
    float gait_duration_slider = 1.5;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_PosePublisher;
    geometry_msgs::msg::PoseStamped m_PoseMessage;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_TargetHeightPublisher;
    std_msgs::msg::Float64 m_TargetHeightMessage;

    struct {
        float scalar = 0.1f;

        bool stamped = true;
        bool gas_button = false;
        bool publish_cmd = true;
        bool enable_publish_on_gas = true;

        bool lock_input = false;

        ImVec2 ang_joy = ImVec2(0, 0), lin_joy = ImVec2(0, 0);
        float roll = 0, pitch = 0, yaw = 0, target_height = 0.2;

    } m_Input;

    // Arm control
};