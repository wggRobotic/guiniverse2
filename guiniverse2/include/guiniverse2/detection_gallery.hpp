#pragma once

#include <memory>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <guiniverse2/gui_image.hpp>
#include <vector>

class ImageGalleryImage
{
public:
    std::unique_ptr<GuiImage> gui_image;
    std::string name;
};

class DetectionGallery
{
public:
    DetectionGallery() = delete;
    DetectionGallery(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, std::string topic);

    void on_gui_frame();

private:

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr json_subscriber;

    std::string image_panel_name;
    std::string json_panel_name;

    std::mutex mutex;
    std::vector<ImageGalleryImage> images;
    std::string json;
    int index;
};