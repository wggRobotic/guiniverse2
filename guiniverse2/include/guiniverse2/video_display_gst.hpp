#pragma once

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <guiniverse2/gui_image.hpp>

class VideoDisplayGST
{
public:
    VideoDisplayGST() = delete;
    VideoDisplayGST(int port, bool fv, bool fh, bool r);
    ~VideoDisplayGST();

    void pull_frame();
    void on_gui_frame();

private:
    GstElement *pipeline;
    GstElement *appsink;
    GstAppSink *sink;

    GstClockTime last_frame_timestamp;

    std::string panel_name;
    GuiImage gui_image;
};