#include "guiniverse2/joystick_input.hpp"
#include "imgui.h"
#include <algorithm>
#include <memory>
#include <quac/quac.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <thread>
#include <guiniverse2/imgui_utils.hpp>
#include <unistd.h>

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
    joystick_input("DualSense Wireless Controller"),
    session_folder("runs/quac_" + get_time_string() + "/"),
    node(std::make_shared<rclcpp::Node>("guiniverse", "quac")),
    callback_group(node->create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
    tf_buffer(node->get_clock()),
    tf_listener(tf_buffer, node, false),
    front_cam(5000, true, true, false),
    gripper_cam(5002, false, false, false),
    back_cam(5001, false, false, false),
    left_cam(5003, false, false, false),
    right_cam(5004, false, false, false),
    thermal_cam(node, callback_group, "thermal_image/compressed", false, false, true),
    hazmat_gallery(node, callback_group, "hazmat_signs", session_folder + "hazmat_signs/"),
    qrcode_gallery(node, callback_group, "qrcodes", session_folder + "qrcodes/"),
    landolt_gallery(node, callback_group, "landolt_cs", session_folder + "landolt_cs/"),
    map_image(false, false, false),
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

    m_IPMessage.data = std::getenv("VIDEO_TARGET_IP") ? std::getenv("VIDEO_TARGET_IP") : "127.0.0.1";

    m_IPPublisher = node->create_publisher<std_msgs::msg::String>("video_target_ip", rclcpp::QoS(2).reliable());

    ip_timer = node->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            m_IPPublisher->publish(m_IPMessage);
        },
        callback_group
    );

    m_Input.gas_button = false;
    m_Input.dual_joy = false;

    m_TwistPublisher = node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_pilot", rclcpp::QoS(2).reliable());

    m_Arm.publish_pose = false;
    m_Arm.target_pos = ImVec2(0.0475f, 0.0288f);
    m_Arm.old_arm = false;
    for (int i = 0; i < 3; i++) { m_Arm.arm_segments[i].index = -1; m_Arm.arm_segments[i].value = 0;}
    
    m_JointStatesSubscriber = node->create_subscription<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(2).reliable(), std::bind(&Quac::jointStateCallback, this, std::placeholders::_1), options);
    m_ArmPosePublisher = node->create_publisher<geometry_msgs::msg::Pose>("ee_pose", rclcpp::QoS(2).reliable());
    m_GripperWidthPublisher = node->create_publisher<std_msgs::msg::Float64>("gripper_width", rclcpp::QoS(2).reliable());

    m_ImuSubscriber = node->create_subscription<sensor_msgs::msg::Imu>(
        "camera_front/imu", 
        rclcpp::QoS(2).best_effort(), 
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(m_SensorData.mutex);
            m_SensorData.imu = *msg;
        },
        options
    );
    
    m_MagneticFieldSubscriber = node->create_subscription<sensor_msgs::msg::MagneticField>(
        "magnetic_field", 
        rclcpp::QoS(2).best_effort(), 
        [this](const sensor_msgs::msg::MagneticField::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(m_SensorData.mutex);
            m_SensorData.magnetic_field = *msg;
        },
        options
    );

#define MAP_SCALE_FACTOR 4
#define TRIM_FACTOR 10

    m_MapSubscriber = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map",
        rclcpp::QoS(1),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            int x_min = msg->info.width, x_max = 0, y_min = msg->info.height, y_max = 0;

            for (int y = 0; y < (int)msg->info.height; y += TRIM_FACTOR)
            {
                for (int x = 0; x < (int)msg->info.width; x += TRIM_FACTOR)
                {
                    if (msg->data[y * msg->info.width + x] != -1)
                    {
                        if (x <= x_min) x_min = x - TRIM_FACTOR, 0;
                        if (x >= x_max) x_max = x + TRIM_FACTOR, (int)msg->info.width;
                        if (y <= y_min) y_min = y - TRIM_FACTOR, 0;
                        if (y >= y_max) y_max = y + TRIM_FACTOR, (int)msg->info.height;
                    }
                }
            }

            x_min = std::max(0, x_min - TRIM_FACTOR);
            x_max = std::min((int)msg->info.width, x_max + TRIM_FACTOR);
            y_min = std::max(0, y_min - TRIM_FACTOR);
            y_max = std::min((int)msg->info.height, y_max + TRIM_FACTOR);


            if (x_min > x_max || y_min > y_max) return;

            cv::Mat image((y_max - y_min) * MAP_SCALE_FACTOR, (x_max - x_min) * MAP_SCALE_FACTOR, CV_8UC3);

            for (uint32_t y = 0; y < image.rows; y++)
            {
                for (uint32_t x = 0; x < image.cols; x++)
                {
                    int8_t occ = msg->data[(y / MAP_SCALE_FACTOR + y_min) * msg->info.width + (x / MAP_SCALE_FACTOR + x_min)];
                    uint8_t value = (occ == -1 ? 127 : 255 - (occ * 255 / 100));

                    image.at<cv::Vec3b>(y, x) = cv::Vec3b(value, value, value);
                }
            }

            cv::imwrite(session_folder + "map.png", image);

            quac_interfaces::msg::DetectedObjectArray objects[3];
            qrcode_gallery.get_objects(objects[0]);
            landolt_gallery.get_objects(objects[1]);
            hazmat_gallery.get_objects(objects[2]);

            tf2::Transform grid_to_map;
            tf2::fromMsg(msg->info.origin, grid_to_map);
            tf2::Transform map_to_grid = grid_to_map.inverse();

            for (auto& array : objects)
            {
                for (auto& object : array.objects)
                {
                    tf2::Vector3 p_grid = map_to_grid * tf2::Vector3(object.pose.position.x , object.pose.position.y, 0.0);

                    int col = (p_grid.x() / msg->info.resolution - x_min) * (float)MAP_SCALE_FACTOR;
                    int row = (p_grid.y() / msg->info.resolution - y_min) * (float)MAP_SCALE_FACTOR;

                    RCLCPP_INFO(node->get_logger(), "drawing circle at %d %d", col, row);

                    cv::circle(
                        image,
                        cv::Point(col, row),
                        MAP_SCALE_FACTOR / 2,
                        cv::Scalar(0, 111, 255),
                        -1
                    );

                    cv::putText(
                        image, 
                        object.header.frame_id, 
                        cv::Point(col - MAP_SCALE_FACTOR, row - MAP_SCALE_FACTOR ),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.4,
                        cv::Scalar(0, 0, 255),
                        1,
                        cv::LINE_AA
                    );
                }
            }

            cv::imwrite(session_folder + "map_annotated.png", image);

            map_image.set_image(image, 1.f);
        },
        options
    );

    running.store(true);
    thread = std::thread([this]() {
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
        executor.add_node(node);
        while (running.load()) executor.spin_some();
    });

    gst_thread = std::thread([this]() {
        while (running.load())
        {
            front_cam.pull_frame();
            gripper_cam.pull_frame();
            back_cam.pull_frame();
            left_cam.pull_frame();
            right_cam.pull_frame();
            usleep(2000*10);
        }  
    });
}

Quac::~Quac()
{
    running.store(false);
    thread.join();
    gst_thread.join();
}

void Quac::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::string joint_names[4] = {"arm_segment_0_joint", "arm_segment_1_joint", "arm_segment_2_joint"};

    std::lock_guard<std::mutex> lock(m_Arm.mutex);

    for (int i = 0; i < (m_Arm.old_arm ? 2 : 3); i++)
    {
        if (m_Arm.arm_segments[i].index != -1)
        {
            if (m_Arm.arm_segments[i].index < msg->name.size())
                if (msg->name[m_Arm.arm_segments[i].index] == joint_names[i])
                {
                    m_Arm.arm_segments[i].value = msg->position[m_Arm.arm_segments[i].index];
                    continue;
                }

            m_Arm.arm_segments[i].index = -1;
        }
        
        if (m_Arm.arm_segments[i].index == -1)
        {
            for (int j = 0; j < msg->name.size(); j++)
            {
                if (msg->name[j] == joint_names[i])
                {
                    m_Arm.arm_segments[i].index = j;
                    m_Arm.arm_segments[i].value = msg->position[j];
                }
            }
        }
    }
    
}


void Quac::on_gui_frame(GLFWwindow* window)
{

    if (ImGui::Begin("Control"))
    {
        m_Input.gas_button = false;
        if (glfwGetKey(window, GLFW_KEY_SPACE)) m_Input.gas_button = true;

        ImGui::Checkbox("Publish cmd", &m_Input.publish_cmd);
        ImGui::Checkbox("Enable publish on gas", &m_Input.enable_publish_on_gas);
        ImGui::Checkbox("dual joy", &m_Input.dual_joy);

        if (m_Input.gas_button && m_Input.enable_publish_on_gas) m_Input.publish_cmd = true;
        if (ImGui::Button("Reset run"))
        {
            session_folder = "runs/quac_" + get_time_string() + "/";
            hazmat_gallery.reset(session_folder + "hazmat_signs/");
            qrcode_gallery.reset(session_folder + "qrcodes/");
            landolt_gallery.reset(session_folder + "landolt_cs/");
        }

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

   
        if (m_Input.publish_cmd)
        {
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

            m_TwistPublisher->publish(m_TwistMessage);
        }

    }
    ImGui::End();

    if (ImGui::Begin("Robotic arm"))
    {
        std::lock_guard<std::mutex> lock(m_Arm.mutex);

        if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) m_Arm.target_pos.y += 0.001f;
        if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) m_Arm.target_pos.y -= 0.001f;
        if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) m_Arm.target_pos.x -= 0.001f;
        if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) m_Arm.target_pos.x += 0.001f;

        m_Arm.publish_pose = false;
        if (glfwGetKey(window, GLFW_KEY_P)) m_Arm.publish_pose = true;
        m_Arm.publish_width = false;
        if (glfwGetKey(window, GLFW_KEY_O)) m_Arm.publish_width = true;

        ImGui::Text("target x: %fm   target y: %fm ", m_Arm.target_pos.x, m_Arm.target_pos.y);
        ImGui::SliderFloat("Gripper width", &m_Arm.gripper_width, 0.0f, 0.08f);
        ImGui::SliderFloat("Target angle", &m_Arm.target_angle, -1.7f, 1.7f);
        if (ImGui::Button("copy gripper angle")) m_Arm.target_angle = m_Arm.arm_segments[0].value + m_Arm.arm_segments[1].value + m_Arm.arm_segments[2].value;
        if (ImGui::Button("zero gripper angle")) m_Arm.target_angle = 0.f;
        ImGui::Checkbox("Old Arm", &m_Arm.old_arm);
        if (ImGui::Button("Base"))
        {
            m_Arm.target_pos = ImVec2(-0.12f, 0.05f);
            m_Arm.target_angle = 0.0;
        }
        if (ImGui::Button("Forward0")) m_Arm.target_pos = ImVec2(0.1f, 0.05f);
        if (ImGui::Button("ButtonSide")) { m_Arm.target_pos = ImVec2(0.134559f, 0.021273f); m_Arm.target_angle = -0.763;} 

        m_Arm.target_pos = ImVec2(std::clamp(m_Arm.target_pos.x, -0.12f, 0.3f), std::clamp(m_Arm.target_pos.y, -0.3f, 0.3f));

        if (m_Arm.target_pos.x < 0.03)
        {
            m_Arm.target_pos.y = std::max(m_Arm.target_pos.y, 0.0f);
            m_Arm.target_angle = std::max(m_Arm.target_angle, 0.0f);
        }

        float scalar = 800.f;
        ImVec2 offset = ImVec2(300.f, 250.f);

        struct {
            float base_front = -0.01f;
            float base_back = -0.09f;
            float base_y = -0.015f;
            float diaginal_down_y = 0.01f;
            float diagonal_x = 0.063;
            float diagonal_y = 0.08f;
        } chassis;

#define chasis_line(x_0, y_0, x_1, y_1) imgui_line(ImVec2(offset.x + (x_0) * scalar, offset.y - (y_0) * scalar), ImVec2(offset.x + (x_1) * scalar, offset.y - (y_1) * scalar), IM_COL32(150, 150, 150, 255), 3.f)

        chasis_line(-1.02, -0.18, 0.98, -0.18);
        chasis_line(chassis.base_back, chassis.base_y, chassis.base_front, chassis.base_y);
        chasis_line(chassis.base_back, chassis.base_y, chassis.base_back, chassis.base_y + chassis.diaginal_down_y);
        chasis_line(chassis.base_back, chassis.base_y + chassis.diaginal_down_y, chassis.base_back - chassis.diagonal_x, chassis.base_y + chassis.diaginal_down_y + chassis.diagonal_y);

        float angles[6];
        float segment_lengths[6];
        if (m_Arm.old_arm)
        {
            angles[0] = M_PIf + m_Arm.arm_segments[0].value;
            angles[1] = - M_PIf / 2;
            angles[2] = - M_PIf / 2 + m_Arm.arm_segments[1].value;

            segment_lengths[0] = 0.1025f;
            segment_lengths[1] = 0.0288f;
            segment_lengths[2] = 0.15f;
        }
        else
        {
            angles[0] = M_PI + m_Arm.arm_segments[0].value;
            angles[1] = m_Arm.arm_segments[1].value;
            angles[2] = - M_PI / 2.0;
            angles[3] = - M_PI / 2 + m_Arm.arm_segments[2].value;
            angles[4] = - M_PI / 2.0;
            angles[5] =  M_PI / 2.0;

            segment_lengths[0] = 0.1f;
            segment_lengths[1] = 0.02f;
            segment_lengths[2] = 0.05f;
            segment_lengths[3] = 0.05f;
            segment_lengths[4] = 0.015f;
            segment_lengths[5] = 0.11f;
        }
        

        ImVec2 joint = offset;

        float angle = 0;

        for ( int i = 0; i < (m_Arm.old_arm ? 3 : 6); i++ )
        {
            angle += angles[i];
            ImVec2 next_joint = ImVec2(cos(angle) * segment_lengths[i] * scalar + joint.x, -sin(angle) * segment_lengths[i] * scalar + joint.y);

            imgui_line(joint, next_joint, IM_COL32(200, 200, 200, 255), 4.f);
    
            joint = next_joint;

            if (i == 2) if (ImGui::Button("copy gripper pos")) m_Arm.target_pos = ImVec2((joint.x - offset.x) / scalar, (joint.y - offset.y) / scalar);
        }

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 panel_pos = ImGui::GetWindowPos();
    
        draw_list->AddCircleFilled(ImVec2(
            offset.x + m_Arm.target_pos.x * scalar + panel_pos.x, 
            offset.y - m_Arm.target_pos.y * scalar + panel_pos.y
        ), 4.f, IM_COL32(255, 0, 0, 255));
    
        {
            if (m_Arm.publish_pose)
            {
                m_ArmPoseMessage.position.x = m_Arm.target_pos.x;
                m_ArmPoseMessage.position.z = m_Arm.target_pos.y;
                m_ArmPoseMessage.orientation.y = m_Arm.target_angle;

                m_ArmPosePublisher->publish(m_ArmPoseMessage);
            }
            if (m_Arm.publish_width)
            {
                m_GripperWidthMessage.data = m_Arm.gripper_width;
                m_GripperWidthPublisher->publish(m_GripperWidthMessage);
            }
        }
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

    if (ImGui::Begin("Map"))
    {
        map_image.draw();
    }
    ImGui::End();
    
    bool dummy;
    
    front_cam.on_gui_frame(&show_front_settings);
    if (show_front_settings) front_cam_overlay.settings_panel();
    front_cam_overlay.draw(m_Input.cmd_values.x, m_Input.cmd_values.y);

    gripper_cam.on_gui_frame(&show_gripper_settings);
    if (show_gripper_settings) gripper_cam_overlay.settings_panel();
    gripper_cam_overlay.draw(m_Input.cmd_values.x, m_Input.cmd_values.y);

    back_cam.on_gui_frame(&show_back_settings);
    if (show_back_settings) back_cam_overlay.settings_panel();
    back_cam_overlay.draw(m_Input.cmd_values.x, m_Input.cmd_values.y);

    left_cam.on_gui_frame(&dummy);
    right_cam.on_gui_frame(&dummy);

    thermal_cam.on_gui_frame(&dummy);
    hazmat_gallery.on_gui_frame();
    qrcode_gallery.on_gui_frame();
    landolt_gallery.on_gui_frame();
}