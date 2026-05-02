#pragma once

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <rclcpp/subscription.hpp>
#include <guiniverse2/gui_image.hpp>

class VideoDisplayROS
{
public:
    VideoDisplayROS() = delete;
    VideoDisplayROS(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, std::string topic, bool fv, bool fh, bool r);

    void on_gui_frame();

private:

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscriber;

    std::string panel_name;
    GuiImage gui_image;
};