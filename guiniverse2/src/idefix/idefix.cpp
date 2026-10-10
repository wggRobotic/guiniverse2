#include "imgui.h"
#include <idefix/idefix.hpp>

#include <guiniverse2/imgui_utils.hpp>
#include <unistd.h>

Idefix::Idefix() : joystick_input("DualSense Wireless Controller"), node(std::make_shared<rclcpp::Node>("guiniverse"))
{
    m_Input.gas_button = false;
    m_TwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 2);
    m_TwistStampedPublisher = node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 2);
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

        m_Input.gas_button = false;
        if (glfwGetKey(window, GLFW_KEY_SPACE)) m_Input.gas_button = true;

        ImGui::Checkbox("Stamped", &m_Input.stamped);
        ImGui::Checkbox("Publish cmd", &m_Input.publish_cmd);
        ImGui::Checkbox("Enable publish on gas", &m_Input.enable_publish_on_gas);

        if (m_Input.gas_button && m_Input.enable_publish_on_gas) m_Input.publish_cmd = true;

        bool invert_cmd = false;

        bool dualsense_wireless = (joystick_input.getDeviceName() == "DualSense Wireless Controller");
        bool android_gamepad = (joystick_input.getDeviceName() == "Android Gamepad");

        ImVec2 ang_joy, lin_joy;

        if (dualsense_wireless || android_gamepad)
        {
            ang_joy.x = -joystick_input.getAxis(1);
            ang_joy.y = -joystick_input.getAxis(0);

            lin_joy.x = -joystick_input.getAxis(3);
            lin_joy.y = -joystick_input.getAxis(2);

            m_Input.gas_button = joystick_input.getButton(dualsense_wireless ? 5 : 7);
            invert_cmd = joystick_input.getButton(dualsense_wireless ? 7 : 9);
        }

        if (glfwGetKey(window, GLFW_KEY_S)) ang_joy.x = -1.f;
        if (glfwGetKey(window, GLFW_KEY_D)) ang_joy.y = -1.f;
        if (glfwGetKey(window, GLFW_KEY_W)) ang_joy.x = 1.f;
        if (glfwGetKey(window, GLFW_KEY_A)) ang_joy.y = 1.f;

        if (glfwGetKey(window, GLFW_KEY_K)) lin_joy.x = -1.f;
        if (glfwGetKey(window, GLFW_KEY_L)) lin_joy.y = -1.f;
        if (glfwGetKey(window, GLFW_KEY_I)) lin_joy.x = 1.f;
        if (glfwGetKey(window, GLFW_KEY_J)) lin_joy.y = 1.f;

        if (ang_joy.x > -0.2f && ang_joy.x < 0.1f) ang_joy.x = 0.f;
        if (ang_joy.y > -0.1f && ang_joy.y < 0.1f) ang_joy.y = 0.f;

        float ang_length = sqrtf(ang_joy.x * ang_joy.x + ang_joy.y * ang_joy.y);
        if (ang_length > 1.0f)
        {
            ang_joy = ImVec2(ang_joy.x / ang_length, ang_joy.y / ang_length);
            ang_length = 1.f;
        }

        if (lin_joy.x > -0.2f && lin_joy.x < 0.1f) lin_joy.x = 0.f;
        if (lin_joy.y > -0.1f && lin_joy.y < 0.1f) lin_joy.y = 0.f;

        float lin_length = sqrtf(lin_joy.x * lin_joy.x + lin_joy.y * lin_joy.y);
        if (lin_length > 1.0f)
        {
            lin_joy = ImVec2(lin_joy.x / lin_length, lin_joy.y / lin_length);
            lin_length = 1.f;
        }

        if (ang_length == 0.0) ang_joy.x = 1.f;
        if (lin_length == 0.0) lin_joy.x = 1.f;

        imgui_joystick("virtual angular joystick", 200.f, ImVec2(0.0f, 0.0f), (ang_joy.x == 0.f && ang_joy.y == 0.f) ? 0 : &ang_joy, (m_Input.gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        ImGui::SameLine(0.0f, 10.0f);

        imgui_joystick("virtual linear joystick", 200.f, ImVec2(0.0f, 0.0f), (lin_joy.x == 0.f && lin_joy.y == 0.f) ? 0 : &lin_joy, (m_Input.gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        ImGui::SameLine(0.0f, 10.0f);

        ImGui::VSliderFloat("##vslider", ImVec2(20, 200), &m_Input.scalar, 0.001f, 0.1f, "%.2f");

        ImVec4 twist_values = {lin_joy.x * ang_joy.x, lin_joy.y * ang_joy.x, ang_joy.y, 0.0};

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
                m_TwistStampedMessage.twist.angular.z = twist_values.z * m_Input.scalar * 2.f;
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

    }
    ImGui::End();
}