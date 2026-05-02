#include <memory>
#include <quac/quac.hpp>

#include <rclcpp/executors.hpp>
#include <thread>
#include <guiniverse2/imgui_utils.hpp>

Quac::Quac() : 
    node(std::make_shared<rclcpp::Node>("guiniverse", "quac")),
    callback_group(node->create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
    front_cam(5000, false, false, false),
    back_cam(5001, false, false, false),
    thermal_cam(node, callback_group, "thermal_image/compressed", false, false, true),
    hazmat_gallery(node, callback_group, "hazmat_signs"),
    qrcode_gallery(node, callback_group, "qrcodes")
{
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group;

    gst_timer = node->create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() {
            front_cam.pull_frame();
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
    m_ArmPosePublisher = node->create_publisher<geometry_msgs::msg::Pose>("ee_pos", 10);

    cmd_timer = node->create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() {
            {
                bool gas = false;
                {
                    std::lock_guard<std::mutex> lock(m_InputMutex);
                    if (gas = m_Input.gas_button)
                    {
                        m_TwistMessage.linear.x =  m_Input.main_axes.y * m_Input.scalar;
                        m_TwistMessage.angular.z = m_Input.main_axes.x * m_Input.scalar * (m_Input.main_axes.y * m_Input.scalar < 0 ? -1.f : 1.f) * 2.f;
                    }
                }
                if (gas) m_TwistPublisher->publish(m_TwistMessage);
            }

            {
                std::lock_guard<std::mutex> lock(m_Arm.mutex);

                if (m_Arm.publish_pose)
                {
                    m_ArmPoseMessage.position.x = m_Arm.target_pose.x;
                    m_ArmPoseMessage.position.z = m_Arm.target_pose.y;

                    m_ArmPosePublisher->publish(m_ArmPoseMessage);
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

        if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) m_Arm.target_pose.y += 0.001f;
        if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) m_Arm.target_pose.y -= 0.001f;
        if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) m_Arm.target_pose.x -= 0.001f;
        if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) m_Arm.target_pose.x += 0.001f;

        m_Arm.publish_pose = false;
        if (glfwGetKey(window, GLFW_KEY_P)) m_Arm.publish_pose = true;

        ImGui::Text("target x: %fm   target y: %fm ", m_Arm.target_pose.x, m_Arm.target_pose.y);

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

        chasis_line(chassis.base_back, chassis.base_y, chassis.base_front, chassis.base_y);
        chasis_line(chassis.base_back, chassis.base_y, chassis.base_back, chassis.base_y + chassis.diaginal_down_y);
        chasis_line(chassis.base_back, chassis.base_y + chassis.diaginal_down_y, chassis.base_back - chassis.diagonal_x, chassis.base_y + chassis.diaginal_down_y + chassis.diagonal_y);

        float angles[3];
        angles[0] = M_PIf + m_Arm.joints[0].value;
        angles[1] = - M_PIf / 2;
        angles[2] = - M_PIf / 2 + m_Arm.joints[1].value;

        float segment_lengths[3] = {0.105f, 0.035f, 0.13f};
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
    
    front_cam.on_gui_frame();
    back_cam.on_gui_frame();
    thermal_cam.on_gui_frame();
    hazmat_gallery.on_gui_frame();
    qrcode_gallery.on_gui_frame();
}