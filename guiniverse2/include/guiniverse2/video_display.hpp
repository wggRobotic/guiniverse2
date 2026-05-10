#pragma once

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <memory>
#include <mutex>
#include <quac_interfaces/msg/bounding_box_array.hpp>
#include <rclcpp/node.hpp>

#include <guiniverse2/gui_image.hpp>

class bounding_box_topic_manager
{
public:
    std::mutex mutex;
    rclcpp::Subscription<quac_interfaces::msg::BoundingBoxArray>::SharedPtr subscriber;
    quac_interfaces::msg::BoundingBoxArray msg;
    bool show;
    std::string topic;
};

class VideoDisplay
{
public:
    VideoDisplay() = delete;
    VideoDisplay(const std::string& name, bool fv, bool fh, bool r, const std::vector<std::string>& bb_topics, rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group);

    void on_gui_frame();

protected:
    GuiImage gui_image;

private:
    std::vector<std::unique_ptr<bounding_box_topic_manager>> box_managers;

    std::string panel_name;
};