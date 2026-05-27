#include "imgui.h"
#include <quac_interfaces/msg/bounding_box_array.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <guiniverse2/gui_image.hpp>

class bounding_box_topic_manager
{
public:
    bounding_box_topic_manager() = delete;
    bounding_box_topic_manager(const std::string& topic, rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group);
    void draw(GuiImage* image, ImU32 color);

    std::mutex mutex;
    rclcpp::Subscription<quac_interfaces::msg::BoundingBoxArray>::SharedPtr subscriber;
    quac_interfaces::msg::BoundingBoxArray box_array;
    bool show;
    std::string topic;
};


class CamOverlay {
public:
    CamOverlay() = delete;
    CamOverlay(
        const std::string& cam_panel_name,
        GuiImage* img, 
        const std::string& cam_name,
        rclcpp::Node::SharedPtr node,
        rclcpp::CallbackGroup::SharedPtr group,
        tf2_ros::Buffer* buffer
    );

    bool project_line_points(
        const tf2::Vector3& a_world,
        const tf2::Vector3& b_world,
        ImVec2& out_a,
        ImVec2& out_b
    );
    void draw(float v_x, float omega_z);
    void settings_panel();
private:

    rclcpp::Node::SharedPtr node;
    std::string panel_name;
    GuiImage* image;
    bounding_box_topic_manager qrcode_manager;
    bounding_box_topic_manager hazmat_manager;
    bounding_box_topic_manager landolt_manager;
    bounding_box_topic_manager paintroller_manager;

    std::string cam_frame;
    tf2_ros::Buffer* tf_buffer;
    bool dynamic_robot_path;
    float dynamic_path_seconds;
    float fixed_path_length;

    tf2::Transform base_to_cam;
};
