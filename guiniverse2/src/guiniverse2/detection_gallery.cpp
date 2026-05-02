#include <guiniverse2/detection_gallery.hpp>
#include <memory>
#include <mutex>

DetectionGallery::DetectionGallery(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, std::string topic)
{
    image_panel_name = "Images from " + topic + "/images";
    json_panel_name = "JSON from " + topic + "/json";
    index = 0;

    rclcpp::SubscriptionOptions options;
    options.callback_group = group;

    image_subscriber = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        topic + "/images",
        10,
        [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
            cv::Mat raw_data(1, msg->data.size(), CV_8UC1, const_cast<unsigned char*>(msg->data.data()));

            cv::Mat image = cv::imdecode(raw_data, cv::IMREAD_COLOR);

            std::lock_guard<std::mutex> lock(mutex);
            
            for (int i = 0; i < images.size(); i++)
            {
                if (images[i].name != msg->header.frame_id) continue;

                images[i].gui_image->set_image(image, 1.f);
                return;
            }

            images.push_back({.gui_image = std::make_unique<GuiImage>(false, false, false), .name = msg->header.frame_id,});
            images[images.size() - 1].gui_image->set_image(image, 1.f);
        },
        options
    );

    json_subscriber = node->create_subscription<std_msgs::msg::String>(
        topic + "/json",
        10,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex);
            
            json = msg->data;
        },
        options
    );
}

void DetectionGallery::on_gui_frame()
{
    if (ImGui::Begin(image_panel_name.c_str()))
    {
        std::lock_guard<std::mutex> lock(mutex);

        if (images.size() == 0) ImGui::Text("No images yet");
        else
        {
            if (ImGui::BeginCombo("Select Image", images[index].name.c_str()))
            {
                for (int i = 0; i < images.size(); i++)
                {
                    if (ImGui::Selectable(images[i].name.c_str(), index == i)) index = i;
                }
                
                ImGui::EndCombo();
            }

            images[index].gui_image->draw();
        };

    }
    ImGui::End();

    if (ImGui::Begin(json_panel_name.c_str()))
    {
        ImGui::Text(json.empty() ? "No JSON yet" : json.c_str());
    }
    ImGui::End();
}