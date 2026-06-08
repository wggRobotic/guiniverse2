#include "imgui.h"
#include <memory>
#include <quac/quac.hpp>

#include <rclcpp/executors.hpp>
#include <thread>
#include <guiniverse2/imgui_utils.hpp>

std::string get_time_string()
{
    auto t = std::time(nullptr);
    std::tm tm;
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tm);
    return std::string(buf);
}

Quac::Quac() : 
    session_folder("runs/quac_" + get_time_string() + "/"),
    node(std::make_shared<rclcpp::Node>("guiniverse", "quac")),
    callback_group(node->create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
    tf_buffer(node->get_clock()),
    tf_listener(tf_buffer, node, false),
    front_cam(5000, true, true, false),
    gripper_cam(5002, false, false, false),
    back_cam(5001, false, false, false),
    thermal_cam(node, callback_group, "thermal_image/compressed", false, false, true),
    hazmat_gallery(node, callback_group, "hazmat_signs", session_folder + "hazmat_signs/"),
    qrcode_gallery(node, callback_group, "qrcodes", session_folder + "qrcodes/"),
    landolt_gallery(node, callback_group, "landolt_cs", session_folder + "landolt_cs/"),
    front_cam_overlay(
        front_cam.panel_name,
        &front_cam.gui_image, 
        "camera_front",
        node,
        callback_group,
        &tf_buffer
    ),
    show_front_settings(false),
    gripper_cam_overlay(
        gripper_cam.panel_name,
        &gripper_cam.gui_image, 
        "camera_gripper",
        node,
        callback_group,
        &tf_buffer
    ),
    show_gripper_settings(false),
    back_cam_overlay(
        back_cam.panel_name,
        &back_cam.gui_image, 
        "camera_back",
        node,
        callback_group,
        &tf_buffer
    ),
    show_back_settings(false)
{
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group;

    gst_timer = node->create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() {
            front_cam.pull_frame();
            gripper_cam.pull_frame();
            back_cam.pull_frame();
        },
        callback_group
    );

    m_IPMessage.data = std::getenv("VIDEO_TARGET_IP") ? std::getenv("VIDEO_TARGET_IP") : "127.0.0.1";

    m_IPPublisher = node->create_publisher<std_msgs::msg::String>("video_target_ip", 10);

    ip_timer = node->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            m_IPPublisher->publish(m_IPMessage);
        },
        callback_group
    );

    m_Input.gas_button = false;

    m_TwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_pilot", 10);

    m_Arm.publish_pose = false;
    for (int i = 0; i < 3; i++) { m_Arm.joints[i].index = -1; m_Arm.joints[i].value = 0;}
    
    m_JointStatesSubscriber = node->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&Quac::jointStateCallback, this, std::placeholders::_1), options);
    m_ArmPosePublisher = node->create_publisher<geometry_msgs::msg::Pose>("ee_pose", 10);
    m_GripperWidthPublisher = node->create_publisher<std_msgs::msg::Float64>("gripper_width", 10);

    cmd_timer = node->create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() {
            {
                bool pub_cmd = false;
                {
                    std::lock_guard<std::mutex> lock(m_InputMutex);
                    pub_cmd = m_Input.publish_cmd; 

                    m_Input.lin_x = m_Input.main_axes.y * m_Input.scalar;
                    m_Input.ang_z = m_Input.main_axes.x * m_Input.scalar * (m_Input.main_axes.y * m_Input.scalar < 0 ? -1.f : 1.f) * 2.f;

                    if (m_Input.gas_button)
                    {
                        m_TwistMessage.linear.x = m_Input.lin_x;
                        m_TwistMessage.angular.z = m_Input.ang_z;
                    }
                    else 
                    {
                        m_TwistMessage.linear.x = 0;
                        m_TwistMessage.angular.z = 0;
                    }
                }
                if (pub_cmd) m_TwistPublisher->publish(m_TwistMessage);
            }

            {
                std::lock_guard<std::mutex> lock(m_Arm.mutex);

                if (m_Arm.publish_pose)
                {
                    m_ArmPoseMessage.position.x = m_Arm.target_pose.x;
                    m_ArmPoseMessage.position.z = m_Arm.target_pose.y;

                    m_ArmPosePublisher->publish(m_ArmPoseMessage);
                }
                if (m_Arm.publish_width)
                {
                    m_GripperWidthMessage.data = m_Arm.gripper_width;
                    m_GripperWidthPublisher->publish(m_GripperWidthMessage);
                }
            }
        },
        callback_group
    );

    m_ImuSubscriber = node->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 
        10, 
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(m_SensorData.mutex);
            m_SensorData.imu = *msg;
        },
        options
    );
    
    m_MagneticFieldSubscriber = node->create_subscription<sensor_msgs::msg::MagneticField>(
        "magnetic_field", 
        10, 
        [this](const sensor_msgs::msg::MagneticField::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(m_SensorData.mutex);
            m_SensorData.magnetic_field = *msg;
        },
        options
    );

    running.store(true);
    thread = std::thread([this]() {
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
        executor.add_node(node);
        while (running.load()) executor.spin_some();
    });
}

Quac::~Quac()
{
    running.store(false);
    thread.join();
}

void Quac::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::string joint_names[3] = {"arm_servo_0_joint", "arm_servo_1_joint", "arm_servo_0_joint"};

    std::lock_guard<std::mutex> lock(m_Arm.mutex);

    for (int i = 0; i < 3; i++)
    {
        if (m_Arm.joints[i].index != -1)
        {
            if (m_Arm.joints[i].index < msg->name.size())
                if (msg->name[m_Arm.joints[i].index] == joint_names[i])
                {
                    m_Arm.joints[i].value = msg->position[m_Arm.joints[i].index];
                    continue;
                }

            m_Arm.joints[i].index = -1;
        }
        
        if (m_Arm.joints[i].index == -1)
        {
            for (int j = 0; j < msg->name.size(); j++)
            {
                if (msg->name[j] == joint_names[i])
                {
                    m_Arm.joints[i].index = j;
                    m_Arm.joints[i].value = msg->position[j];
                }
            }
        }
    }
    
}


void Quac::on_gui_frame(GLFWwindow* window)
{
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        m_Input.main_axes = ImVec2(0.f, 0.f);

        m_Input.gas_button = false;

        if (glfwGetKey(window, GLFW_KEY_A)) m_Input.main_axes.x = 1.f;
        else if (glfwGetKey(window, GLFW_KEY_D)) m_Input.main_axes.x = -1.f;
        if (glfwGetKey(window, GLFW_KEY_W)) m_Input.main_axes.y = 1.f;
        else if (glfwGetKey(window, GLFW_KEY_S)) m_Input.main_axes.y = -1.f;

        float length = sqrtf(m_Input.main_axes.x * m_Input.main_axes.x + m_Input.main_axes.y * m_Input.main_axes.y);
        if (length > 1.0f) m_Input.main_axes = ImVec2(m_Input.main_axes.x / length, m_Input.main_axes.y / length);

        m_Input.gas_button |= glfwGetKey(window, GLFW_KEY_SPACE);
    }

    if (ImGui::Begin("Control"))
    {
        std::lock_guard<std::mutex> lock(m_InputMutex);

        ImGui::Checkbox("Publish cmd", &m_Input.publish_cmd);
        if (ImGui::Button("Reset run"))
        {
            session_folder = "runs/quac_" + get_time_string() + "/";
            hazmat_gallery.reset(session_folder + "hazmat_signs/");
            qrcode_gallery.reset(session_folder + "qrcodes/");
            landolt_gallery.reset(session_folder + "landolt_cs/");
        }

        ImVec2 pos = ImGui::GetCursorScreenPos();

        m_Input.main_axes = imgui_joystick("virtual joystick", 200.f, ImVec2(0.2f, 0.2f), (m_Input.main_axes.x == 0.f && m_Input.main_axes.y == 0.f) ? 0 : &m_Input.main_axes, (m_Input.gas_button ? IM_COL32(150, 150, 150, 255) : (80, 80, 80, 255)));

        ImGui::SetCursorScreenPos(ImVec2(pos.x + 220.f, pos.y + ImGui::GetStyle().ItemSpacing.y));

        ImGui::VSliderFloat("##vslider", ImVec2(20, 200), &m_Input.scalar, 0.0f, 1.0f, "%.2f");

        //ImGui::SetCursorScreenPos(ImVec2(pos.x + 260.f, pos.y + ImGui::GetStyle().ItemSpacing.y));

    }
    ImGui::End();

    if (ImGui::Begin("Robotic arm"))
    {
        std::lock_guard<std::mutex> lock(m_Arm.mutex);

        if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS && m_Arm.target_pose.y < 0.3) m_Arm.target_pose.y += 0.001f;
        if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS && m_Arm.target_pose.y > -0.3) m_Arm.target_pose.y -= 0.001f;
        if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS && m_Arm.target_pose.x > 0) m_Arm.target_pose.x -= 0.001f;
        if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS && m_Arm.target_pose.x < 0.3) m_Arm.target_pose.x += 0.001f;

        m_Arm.publish_pose = false;
        if (glfwGetKey(window, GLFW_KEY_P)) m_Arm.publish_pose = true;
        m_Arm.publish_width = false;
        if (glfwGetKey(window, GLFW_KEY_O)) m_Arm.publish_width = true;

        ImGui::Text("target x: %fm   target y: %fm ", m_Arm.target_pose.x, m_Arm.target_pose.y);
        ImGui::SliderFloat("Gripper width", &m_Arm.gripper_width, 0.0f, 0.08f);

        float scalar = 800.f;
        ImVec2 offset = ImVec2(300.f, 250.f);

        struct {
            float base_front = 0.01f;
            float base_back = -0.07f;
            float base_y = -0.015f;
            float diaginal_down_y = 0.01f;
            float diagonal_x = 0.065;
            float diagonal_y = 0.08f;
        } chassis;

#define chasis_line(x_0, y_0, x_1, y_1) imgui_line(ImVec2(offset.x + (x_0) * scalar, offset.y - (y_0) * scalar), ImVec2(offset.x + (x_1) * scalar, offset.y - (y_1) * scalar), IM_COL32(150, 150, 150, 255), 3.f)

        chasis_line(-1, -0.18, 1, -0.18);
        chasis_line(chassis.base_back, chassis.base_y, chassis.base_front, chassis.base_y);
        chasis_line(chassis.base_back, chassis.base_y, chassis.base_back, chassis.base_y + chassis.diaginal_down_y);
        chasis_line(chassis.base_back, chassis.base_y + chassis.diaginal_down_y, chassis.base_back - chassis.diagonal_x, chassis.base_y + chassis.diaginal_down_y + chassis.diagonal_y);

        float angles[3];
        angles[0] = M_PIf + m_Arm.joints[0].value;
        angles[1] = - M_PIf / 2;
        angles[2] = - M_PIf / 2 + m_Arm.joints[1].value;

        float segment_lengths[3] = {0.1025f, 0.0288f, 0.15f};
        bool received[3] = {m_Arm.joints[0].index != -1, m_Arm.joints[0].index != -1, m_Arm.joints[1].index != -1};

        ImVec2 joint = offset;

        float angle = 0;

        for ( int i = 0; i < 3; i++ )
        {
            angle += angles[i];
            ImVec2 next_joint = ImVec2(cos(angle) * segment_lengths[i] * scalar + joint.x, -sin(angle) * segment_lengths[i] * scalar + joint.y);

            imgui_line(joint, next_joint, IM_COL32(200, 200, 200, 255), 4.f);
    
            joint = next_joint;
        }

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 panel_pos = ImGui::GetWindowPos();
    
        draw_list->AddCircleFilled(ImVec2(
                offset.x + m_Arm.target_pose.x * scalar + panel_pos.x, 
                offset.y - m_Arm.target_pose.y * scalar + panel_pos.y
            ), 4.f, IM_COL32(255, 0, 0, 255));
    
    }
    ImGui::End();

    if (ImGui::Begin("Sensor Data"))
    {
        std::lock_guard<std::mutex> lock(m_SensorData.mutex);

        char buf[256];
        snprintf(buf, sizeof(buf), 
            "magnetic field:      x: %10.2f   y: %10.2f   z: %10.2f uT", 
            m_SensorData.magnetic_field.magnetic_field.x, 
            m_SensorData.magnetic_field.magnetic_field.y, 
            m_SensorData.magnetic_field.magnetic_field.z
        );
        ImGui::Text(buf);

        snprintf(buf, sizeof(buf), 
            "acceleration:        x: %10.2f   y: %10.2f   z: %10.2f m/s^2", 
            m_SensorData.imu.linear_acceleration.x, 
            m_SensorData.imu.linear_acceleration.y, 
            m_SensorData.imu.linear_acceleration.z
        );

        ImGui::Text(buf);

        snprintf(buf, sizeof(buf), 
            "angular velocity:    x: %10.2f   y: %10.2f   z: %10.2f degrees/s", 
            m_SensorData.imu.angular_velocity.x, 
            m_SensorData.imu.angular_velocity.y, 
            m_SensorData.imu.angular_velocity.z
        );

        ImGui::Text(buf);
    }
    ImGui::End();
    
    front_cam.on_gui_frame(&show_front_settings);
    if (show_front_settings) front_cam_overlay.settings_panel();
    front_cam_overlay.draw(m_Input.lin_x, m_Input.ang_z);

    gripper_cam.on_gui_frame(&show_gripper_settings);
    if (show_gripper_settings) gripper_cam_overlay.settings_panel();
    gripper_cam_overlay.draw(m_Input.lin_x, m_Input.ang_z);

    back_cam.on_gui_frame(&show_back_settings);
    if (show_back_settings) back_cam_overlay.settings_panel();
    back_cam_overlay.draw(m_Input.lin_x, m_Input.ang_z);

    bool dummy;
    thermal_cam.on_gui_frame(&dummy);
    hazmat_gallery.on_gui_frame();
    qrcode_gallery.on_gui_frame();
    landolt_gallery.on_gui_frame();
}