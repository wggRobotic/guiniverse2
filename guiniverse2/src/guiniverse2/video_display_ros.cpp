#include "guiniverse2/video_display.hpp"
#include <guiniverse2/video_display_ros.hpp>

VideoDisplayROS::VideoDisplayROS(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, std::string topic, bool fv, bool fh, bool r, const std::vector<std::string>& bb_topics) : VideoDisplay("Ros2 topic " + topic, fv, fh, r, bb_topics, node, group)
{
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;

    subscriber = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        topic,
        10,
        [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
            cv::Mat raw_data(1, msg->data.size(), CV_8UC1, const_cast<unsigned char*>(msg->data.data()));

            cv::Mat image = cv::imdecode(raw_data, cv::IMREAD_COLOR);
            gui_image.set_image(image, 1.f);
        },
        options
    );
}