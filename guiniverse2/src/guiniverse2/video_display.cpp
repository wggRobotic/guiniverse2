#include <cstdio>
#include <guiniverse2/video_display.hpp>
#include <memory>

VideoDisplay::VideoDisplay(const std::string& name, bool fv, bool fh, bool r, const std::vector<std::string>& bb_topics, rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group) : gui_image(fv, fh, r)
{
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;

    for (int i = 0; i < bb_topics.size(); i++)
    {
        box_managers.emplace_back(std::make_unique<bounding_box_topic_manager>());

        box_managers[i]->subscriber = node->create_subscription<quac_interfaces::msg::BoundingBoxArray>(
            bb_topics[i], 
            10, 
            [this, i](const quac_interfaces::msg::BoundingBoxArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(box_managers[i]->mutex);
                box_managers[i]->msg = *msg;
            },
            options
        );
    }

    panel_name = name;
    show_bb = true;
}

void VideoDisplay::on_gui_frame()
{
    if (ImGui::Begin(panel_name.c_str()))
    {
        if (box_managers.size() > 0) ImGui::Checkbox("Bounding boxes", &show_bb);
        gui_image.draw();
        for (int i = 0; i < box_managers.size(); i++)
        {
            std::lock_guard<std::mutex> lock(box_managers[i]->mutex);
            if (show_bb) for (int j = 0; j < box_managers[i]->msg.boxes.size(); j++) gui_image.draw_bounding_box(box_managers[i]->msg.boxes[j]);
        }
    }
    ImGui::End();
}