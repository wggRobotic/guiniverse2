#include <guiniverse2/video_display_ros.hpp>

VideoDisplayROS::VideoDisplayROS(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, std::string topic, bool fv, bool fh, bool r) : gui_image(fv, fh, r)
{
    panel_name = "Ros2 topic " + topic;

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

void VideoDisplayROS::on_gui_frame()
{
    if (ImGui::Begin(panel_name.c_str()))
    {
        gui_image.draw();
    }
    ImGui::End();
}