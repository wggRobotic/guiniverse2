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
    void on_gui_frame(bool* settings_flag);

    GuiImage gui_image;
    std::string panel_name;

private:
    GstElement *pipeline;
    GstElement *appsink;
    GstAppSink *sink;

    GstClockTime last_frame_timestamp;
};