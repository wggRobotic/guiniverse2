#pragma once

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <guiniverse2/video_display.hpp>

class VideoDisplayGST : public VideoDisplay
{
public:
    VideoDisplayGST() = delete;
    VideoDisplayGST(int port, bool fv, bool fh, bool r, const std::vector<std::string>& bb_topics, rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group);
    ~VideoDisplayGST();

    void pull_frame();

private:
    GstElement *pipeline;
    GstElement *appsink;
    GstAppSink *sink;

    GstClockTime last_frame_timestamp;
};