#pragma once

#include <rclcpp/node.hpp>
#include <guiniverse2/guiniverse2.hpp>
#include <guiniverse2/video_display_gst.hpp>
#include <guiniverse2/video_display_ros.hpp>
#include <guiniverse2/detection_gallery.hpp>

#include <mutex>
#include <rclcpp/timer.hpp>
#include <thread>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

struct arm_joint
{
    int index;
    double value;
};

class Quac : public RobotController
{
public:
    Quac();
    ~Quac();

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void on_gui_frame(GLFWwindow* window) override;
private:

    std::string session_folder;

    rclcpp::Node::SharedPtr node;
    rclcpp::CallbackGroup::SharedPtr callback_group;

    VideoDisplayGST front_cam;
    VideoDisplayGST back_cam;
    VideoDisplayROS thermal_cam;

    DetectionGallery hazmat_gallery;
    DetectionGallery qrcode_gallery;

    rclcpp::TimerBase::SharedPtr gst_timer;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TwistPublisher;
    geometry_msgs::msg::Twist m_TwistMessage;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_ArmPosePublisher;
    geometry_msgs::msg::Pose m_ArmPoseMessage;
    rclcpp::TimerBase::SharedPtr cmd_timer;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_IPPublisher;
    std_msgs::msg::String m_IPMessage;
    rclcpp::TimerBase::SharedPtr ip_timer;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_ImuSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr m_MagneticFieldSubscriber;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_JointStatesSubscriber;

    std::mutex m_InputMutex;

    struct {
        ImVec2 main_axes = ImVec2(0.f, 0.f);
        float scalar = 0.5f;
        float joystick_scalar = 0.5f;

        bool gas_button = false;
        bool publish_cmd = true;
    } m_Input;

    struct
    {
        struct arm_joint joints[3];

        ImVec2 target_pose;
        bool publish_pose;

        std::mutex mutex;
    } m_Arm;

    struct
    {
        std::mutex mutex;

        sensor_msgs::msg::Imu imu;
        sensor_msgs::msg::MagneticField magnetic_field;
    } m_SensorData;

    std::atomic<bool> running;
    std::thread thread;   
};