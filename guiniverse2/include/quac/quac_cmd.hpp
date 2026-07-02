#pragma once

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "guiniverse2/gui_image.hpp"
#include "imgui.h"
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/node.hpp>
#include <guiniverse2/guiniverse2.hpp>
#include <guiniverse2/joystick_input.hpp>

#include <mutex>
#include <rclcpp/timer.hpp>
#include <thread>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class QuacCmd : public RobotController
{
public:
    QuacCmd();
    ~QuacCmd();

    void on_gui_frame(GLFWwindow* window) override;
private:

    JoystickInput joystick_input;

    rclcpp::Node::SharedPtr node;
    rclcpp::CallbackGroup::SharedPtr callback_group;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_TwistPublisher;
    geometry_msgs::msg::TwistStamped m_TwistMessage;
    rclcpp::TimerBase::SharedPtr cmd_timer;
    std::atomic<bool> running;
    std::thread thread;

    std::mutex m_InputMutex;

    struct {
        ImVec2 cmd_values = ImVec2(0, 0);

        float scalar = 0.5f;

        bool gas_button = false;
        bool publish_cmd = true;
        bool enable_publish_on_gas = true;
        bool dual_joy = true;
    } m_Input;
};