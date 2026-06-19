#pragma once

#include "quac_interfaces/msg/detected_object_array.hpp"
#include <memory>
#include <mutex>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <guiniverse2/gui_image.hpp>
#include <quac_interfaces/msg/detected_object_array.hpp>
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
    DetectionGallery(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, const std::string& topic, const std::string folder);

    void on_gui_frame();
    void reset(const std::string& folder);
    void get_objects(quac_interfaces::msg::DetectedObjectArray& objects);

private:

    std::string session_folder;
    std::string image_folder;
    std::mutex folder_mutex;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr json_subscriber;
    rclcpp::Subscription<quac_interfaces::msg::DetectedObjectArray>::SharedPtr object_subscriber;
    quac_interfaces::msg::DetectedObjectArray object_array;
    std::mutex object_array_mutex;

    std::string folder_path;

    std::string image_panel_name;
    std::string json_panel_name;

    std::mutex mutex;
    std::vector<ImageGalleryImage> images;
    std::string json;
    int index;
};