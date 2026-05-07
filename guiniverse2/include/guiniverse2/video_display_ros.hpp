#pragma once

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <rclcpp/subscription.hpp>
#include <guiniverse2/video_display.hpp>

class VideoDisplayROS : public VideoDisplay
{
public:
    VideoDisplayROS() = delete;
    VideoDisplayROS(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, std::string topic, bool fv, bool fh, bool r, const std::vector<std::string>& bb_topics);

private:

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscriber;
};