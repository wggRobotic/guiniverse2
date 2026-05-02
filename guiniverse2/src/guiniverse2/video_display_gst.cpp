#include <guiniverse2/video_display_gst.hpp>

VideoDisplayGST::VideoDisplayGST(int port, bool fv, bool fh, bool r) : gui_image(fv, fh, r)
{
    panel_name = "GStreamer UDP port " + std::to_string(port);

    std::string launch_string =
        "udpsrc port=" + std::to_string(port) + " "
        "caps=\"application/x-rtp, media=video, encoding-name=H264, payload=96\" ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        "video/x-raw,format=BGR ! queue max-size-buffers=1 leaky=downstream ! "
        "appsink name=gst_sink";

    printf("Launching: %s\n", launch_string.c_str());

    pipeline = gst_parse_launch(launch_string.c_str(), nullptr);
    if (!pipeline) { printf("Failed to create pipeline\n"); return;}

    appsink = gst_bin_get_by_name(GST_BIN(pipeline), "gst_sink");
    sink = GST_APP_SINK(appsink);

    // Configure appsink for low-latency, dropping old frames
    g_object_set(sink,
        "emit-signals", TRUE,
        "max-buffers", 1,
        "drop", TRUE,
        nullptr
    );

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

}

VideoDisplayGST::~VideoDisplayGST()
{
    if (!pipeline) return;
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}

void VideoDisplayGST::pull_frame()
{
    if (!pipeline) return;

    GstSample* sample = gst_app_sink_try_pull_sample(sink, 10000000); // 10ms timeout

    if (!sample) return;

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    GstClockTime timestamp = GST_BUFFER_PTS(buffer);

    if (caps && (timestamp != last_frame_timestamp)) {
        GstStructure* structure = gst_caps_get_structure(caps, 0);
        int width = 0, height = 0, par_num = 1, par_den = 1;

        if (structure &&
            gst_structure_get_int(structure, "width", &width) &&
            gst_structure_get_int(structure, "height", &height) &&
            gst_structure_get_fraction(structure, "pixel-aspect-ratio", &par_num, &par_den)
        ) {

            GstMapInfo map;
            if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
                cv::Mat wrapper_mat(height, width, CV_8UC3, (void*)map.data);
                cv::Mat image_mat;
                wrapper_mat.copyTo(image_mat);

                gui_image.set_image(image_mat, (float)par_num/(float)par_den);
                last_frame_timestamp = timestamp;
                gst_buffer_unmap(buffer, &map);
            }
        }
    }

    gst_sample_unref(sample);
}

void VideoDisplayGST::on_gui_frame()
{
    if (ImGui::Begin(panel_name.c_str()))
    {
        gui_image.draw();
    }
    ImGui::End();
}