#include "imgui.h"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <idefix/idefix.hpp>

#include <guiniverse2/imgui_utils.hpp>
#include <unistd.h>

Idefix::Idefix() : joystick_input("DualSense Wireless Controller"), node(std::make_shared<rclcpp::Node>("guiniverse"))
{
    m_Input.gas_button = false;
    m_TwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 2);
    m_TwistStampedPublisher = node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 2);
    m_GaitModePublisher = node->create_publisher<std_msgs::msg::String>("gait_mode", 2);
    m_GaitDurationPublisher = node->create_publisher<std_msgs::msg::Float64>("gait_duration", 2);
}

Idefix::~Idefix()
{

}

// called every gui frame

void Idefix::on_gui_frame(GLFWwindow* window)
{

    // Driving controll

    joystick_input.update();
    joystick_input.ImGuiPanel("Jostick Input");

    if (ImGui::Begin("Control"))
    {
        // gathering input and drawing scene

        if (glfwGetKey(window, GLFW_KEY_Z)) m_Input.lock_input = true;
        ImGui::Checkbox("Lock Input", &m_Input.lock_input);
        ImGui::Checkbox("Stamped", &m_Input.stamped);
        ImGui::Checkbox("Publish cmd", &m_Input.publish_cmd);
        ImGui::Checkbox("Enable publish on gas", &m_Input.enable_publish_on_gas);

        bool invert_cmd = false;

        bool dualsense_wireless = (joystick_input.getDeviceName() == "DualSense Wireless Controller");
        bool android_gamepad = (joystick_input.getDeviceName() == "Android Gamepad");

        if (!m_Input.lock_input)
        {
            m_Input.gas_button = false;
            if (glfwGetKey(window, GLFW_KEY_SPACE)) m_Input.gas_button = true;

            if (m_Input.gas_button && m_Input.enable_publish_on_gas) m_Input.publish_cmd = true;

            m_Input.ang_joy = ImVec2(0, 0); m_Input.lin_joy = ImVec2(0, 0);

            if (dualsense_wireless || android_gamepad)
            {
                m_Input.ang_joy.x = -joystick_input.getAxis(1);
                m_Input.ang_joy.y = -joystick_input.getAxis(0);

                m_Input.lin_joy.x = -joystick_input.getAxis(3);
                m_Input.lin_joy.y = -joystick_input.getAxis(2);

                if (joystick_input.getButton(dualsense_wireless ? 5 : 7)) m_Input.gas_button = true;
                invert_cmd = joystick_input.getButton(dualsense_wireless ? 7 : 9);
            }

            if (glfwGetKey(window, GLFW_KEY_S)) m_Input.ang_joy.x = -1.f;
            if (glfwGetKey(window, GLFW_KEY_D)) m_Input.ang_joy.y = -1.f;
            if (glfwGetKey(window, GLFW_KEY_W)) m_Input.ang_joy.x = 1.f;
            if (glfwGetKey(window, GLFW_KEY_A)) m_Input.ang_joy.y = 1.f;

            if (glfwGetKey(window, GLFW_KEY_K)) m_Input.lin_joy.x = -1.f;
            if (glfwGetKey(window, GLFW_KEY_L)) m_Input.lin_joy.y = -1.f;
            if (glfwGetKey(window, GLFW_KEY_I)) m_Input.lin_joy.x = 1.f;
            if (glfwGetKey(window, GLFW_KEY_J)) m_Input.lin_joy.y = 1.f;

            if (m_Input.ang_joy.x > -0.2f && m_Input.ang_joy.x < 0.1f) m_Input.ang_joy.x = 0.f;
            if (m_Input.ang_joy.y > -0.1f && m_Input.ang_joy.y < 0.1f) m_Input.ang_joy.y = 0.f;

            float ang_length = sqrtf(m_Input.ang_joy.x * m_Input.ang_joy.x + m_Input.ang_joy.y * m_Input.ang_joy.y);
            if (ang_length > 1.0f)
            {
                m_Input.ang_joy = ImVec2(m_Input.ang_joy.x / ang_length, m_Input.ang_joy.y / ang_length);
                ang_length = 1.f;
            }

            if (m_Input.lin_joy.x > -0.2f && m_Input.lin_joy.x < 0.1f) m_Input.lin_joy.x = 0.f;
            if (m_Input.lin_joy.y > -0.1f && m_Input.lin_joy.y < 0.1f) m_Input.lin_joy.y = 0.f;

            float lin_length = sqrtf(m_Input.lin_joy.x * m_Input.lin_joy.x + m_Input.lin_joy.y * m_Input.lin_joy.y);
            if (lin_length > 1.0f)
            {
                m_Input.lin_joy = ImVec2(m_Input.lin_joy.x / lin_length, m_Input.lin_joy.y / lin_length);
                lin_length = 1.f;
            }

            if (ang_length == 0.0) m_Input.ang_joy.x = 1.f;
            if (lin_length == 0.0) m_Input.lin_joy.x = 1.f;
        }

        imgui_joystick("virtual angular joystick", 200.f, ImVec2(0.0f, 0.0f), (m_Input.ang_joy.x == 0.f && m_Input.ang_joy.y == 0.f) ? 0 : &m_Input.ang_joy, (m_Input.gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        ImGui::SameLine(0.0f, 10.0f);

        imgui_joystick("virtual linear joystick", 200.f, ImVec2(0.0f, 0.0f), (m_Input.lin_joy.x == 0.f && m_Input.lin_joy.y == 0.f) ? 0 : &m_Input.lin_joy, (m_Input.gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        ImGui::SameLine(0.0f, 10.0f);

        ImGui::VSliderFloat("##vslider", ImVec2(20, 200), &m_Input.scalar, 0.05f, 0.3f, "%.2f");

        ImGui::SameLine(0.0f, 10.0f);

        ImGui::VSliderFloat("##vslider_gait_duration", ImVec2(20, 200), &gait_duration_slider, 0.3f, 2.0f, "%.2f");

        ImVec4 twist_values = {m_Input.lin_joy.x * m_Input.ang_joy.x, m_Input.lin_joy.y * m_Input.ang_joy.x, m_Input.ang_joy.y, 0.0};

        if (twist_values.x < 0.f) twist_values.z *= -1;

        if (invert_cmd)
        {
            twist_values.x *= -1;
            twist_values.y *= -1;
        }

        // publishing twist message if enabled
   
        if (m_Input.publish_cmd)
        {
            m_TwistStampedMessage.header.frame_id = "base_link";
            m_TwistStampedMessage.header.stamp = node->now();

            if (m_Input.gas_button)
            {
                m_TwistStampedMessage.twist.linear.x = twist_values.x * m_Input.scalar;
                m_TwistStampedMessage.twist.linear.y = twist_values.y * m_Input.scalar;
                m_TwistStampedMessage.twist.angular.z = twist_values.z * m_Input.scalar * 4.f;
            }
            else 
            {
                m_TwistStampedMessage.twist.linear.x = 0;
                m_TwistStampedMessage.twist.linear.y = 0;
                m_TwistStampedMessage.twist.angular.z = 0;
            }

            if (m_Input.stamped) m_TwistStampedPublisher->publish(m_TwistStampedMessage);
            else m_TwistPublisher->publish(m_TwistStampedMessage.twist);
        }

        const char* gait_names[] = {"trot", "walk", "amble"};

        if (ImGui::BeginCombo("Gait Mode", gait_names[gait_index]))
        {
            for (int i = 0; i < sizeof(gait_names)/sizeof(gait_names[0]); i++)
                if (ImGui::Selectable(gait_names[i], gait_index == i)) gait_index = i;
            
            ImGui::EndCombo();
        }

        m_GaitModeMessage.data = std::string(gait_names[gait_index]);
        m_GaitModePublisher->publish(m_GaitModeMessage);

        m_GaitDurationMessage.data = gait_duration_slider;
        m_GaitDurationPublisher->publish(m_GaitDurationMessage);
    }
    ImGui::End();
}