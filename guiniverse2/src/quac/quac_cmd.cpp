#include "guiniverse2/joystick_input.hpp"
#include "imgui.h"
#include <algorithm>
#include <memory>
#include <quac/quac_cmd.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <thread>
#include <guiniverse2/imgui_utils.hpp>
#include <unistd.h>

QuacCmd::QuacCmd() : 
    joystick_input("DualSense Wireless Controller"),
    node(std::make_shared<rclcpp::Node>("guiniverse", "quac")),
    callback_group(node->create_callback_group(rclcpp::CallbackGroupType::Reentrant))
{
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group;

    m_Input.gas_button = false;
    m_Input.dual_joy = false;

    m_TwistPublisher = node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_pilot", rclcpp::QoS(2).reliable());

    cmd_timer = node->create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() {
            {
                bool pub_cmd = false;
                {
                    std::lock_guard<std::mutex> lock(m_InputMutex);
                    pub_cmd = m_Input.publish_cmd; 

                    m_TwistMessage.header.frame_id = "base_link";
                    m_TwistMessage.header.stamp = node->now();

                    if (m_Input.gas_button)
                    {
                        m_TwistMessage.twist.linear.x = m_Input.cmd_values.x;
                        m_TwistMessage.twist.angular.z = m_Input.cmd_values.y;
                    }
                    else 
                    {
                        m_TwistMessage.twist.linear.x = 0;
                        m_TwistMessage.twist.angular.z = 0;
                    }
                }
                if (pub_cmd) m_TwistPublisher->publish(m_TwistMessage);
            }
        },
        callback_group
    );

    running.store(true);
    thread = std::thread([this]() {
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
        executor.add_node(node);
        while (running.load()) executor.spin_some();
    });
}

QuacCmd::~QuacCmd()
{
    running.store(false);
    thread.join();
}


void QuacCmd::on_gui_frame(GLFWwindow* window)
{

    if (ImGui::Begin("Control"))
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        m_Input.gas_button = false;
        if (glfwGetKey(window, GLFW_KEY_SPACE)) m_Input.gas_button = true;

        ImGui::Checkbox("Publish cmd", &m_Input.publish_cmd);
        ImGui::Checkbox("Enable publish on gas", &m_Input.enable_publish_on_gas);
        ImGui::Checkbox("dual joy", &m_Input.dual_joy);

        if (m_Input.gas_button && m_Input.enable_publish_on_gas) m_Input.publish_cmd = true;
        m_Input.cmd_values = ImVec2(0.f, 0.f);
        ImVec2 forward_joy = ImVec2(0.f, 0.f), backward_joy = ImVec2(0.f, 0.f);

        joystick_input.update();
        joystick_input.ImGuiPanel("Jostick Input");

        bool invert_cmd = false;

        if (joystick_input.getDeviceName() == "DualSense Wireless Controller")
        {
            forward_joy.x = -joystick_input.getAxis(1);
            forward_joy.y = -joystick_input.getAxis(0);

            backward_joy.x = -joystick_input.getAxis(4);
            backward_joy.y = -joystick_input.getAxis(3);

            if (joystick_input.getButton(5)) m_Input.gas_button = true;
            if (joystick_input.getButton(7)) invert_cmd = true;
        }

        if (glfwGetKey(window, GLFW_KEY_S)) forward_joy.x = -1.f;
        if (glfwGetKey(window, GLFW_KEY_D)) forward_joy.y = -1.f;
        if (glfwGetKey(window, GLFW_KEY_W)) forward_joy.x = 1.f;
        if (glfwGetKey(window, GLFW_KEY_A)) forward_joy.y = 1.f;

        if (glfwGetKey(window, GLFW_KEY_G)) backward_joy.x = -1.f;
        if (glfwGetKey(window, GLFW_KEY_H)) backward_joy.y = -1.f;
        if (glfwGetKey(window, GLFW_KEY_T)) backward_joy.x = 1.f;
        if (glfwGetKey(window, GLFW_KEY_F)) backward_joy.y = 1.f;

        float forward_length = sqrtf(forward_joy.x * forward_joy.x + forward_joy.y * forward_joy.y);
        if (forward_length > 1.0f) forward_joy = ImVec2(forward_joy.x / forward_length, forward_joy.y / forward_length);
        
        float backward_length = sqrtf(backward_joy.x * backward_joy.x + backward_joy.y * backward_joy.y);
        if (backward_length > 1.0f) backward_joy = ImVec2(backward_joy.x / backward_length, backward_joy.y / backward_length);

        if (forward_joy.x > -0.2f && forward_joy.x < 0.1f) forward_joy.x = 0.f;
        if (forward_joy.y > -0.1f && forward_joy.y < 0.1f) forward_joy.y = 0.f;

        if (backward_joy.x > -0.1f && backward_joy.x < 0.1f) backward_joy.x = 0.f;
        if (backward_joy.y > -0.1f && backward_joy.y < 0.1f) backward_joy.y = 0.f;

        if (m_Input.dual_joy)
        {
            if (forward_joy.x != 0.f || forward_joy.y != 0.f)
            {
                m_Input.cmd_values = forward_joy;
                if (m_Input.cmd_values.x < 0.f) m_Input.cmd_values.x = 0.f;
            }
            else
            {
                m_Input.cmd_values = backward_joy;
                m_Input.cmd_values.y *= -1;
                if (m_Input.cmd_values.x > 0.f) m_Input.cmd_values.x = 0.f;
            }
        }
        else
        {
            m_Input.cmd_values = forward_joy;
            if (m_Input.cmd_values.x < 0.f) m_Input.cmd_values.y *= -1;
        }

        if (invert_cmd)
        {
            m_Input.cmd_values.x *=-1;
        }

        ImVec2 pos = ImGui::GetCursorScreenPos();

        imgui_joystick("virtual forwards joystick", 200.f, ImVec2(0.0f, 0.0f), (forward_joy.x == 0.f && forward_joy.y == 0.f) ? 0 : &forward_joy, (m_Input.gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        ImGui::SameLine(0.0f, 10.0f);

        imgui_joystick("virtual backwards joystick", 200.f, ImVec2(0.0f, 0.0f), (backward_joy.x == 0.f && backward_joy.y == 0.f) ? 0 : &backward_joy, (m_Input.gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        ImGui::SameLine(0.0f, 10.0f);

        ImGui::VSliderFloat("##vslider", ImVec2(20, 200), &m_Input.scalar, 0.0f, 1.0f, "%.2f");

        m_Input.cmd_values.x *= m_Input.scalar;
        m_Input.cmd_values.y *= m_Input.scalar * 2.f;

    }
    ImGui::End();
}