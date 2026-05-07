#pragma once

#include "quac_interfaces/msg/bounding_box.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <imgui.h>
#include <quac_interfaces/msg/bounding_box.hpp>

class GuiImage
{
public:
    GuiImage() = delete;
    GuiImage(bool fv, bool fh, bool r);
    ~GuiImage();

    void draw();
    void draw_bounding_box(quac_interfaces::msg::BoundingBox& box);
    void set_image(cv::Mat& mat, float mat_par);
    ImVec2 uv_calc(ImVec2 uv);

private:
    std::mutex mutex;
    cv::Mat image;

    unsigned int gl_texture;
    unsigned int texture_width;
    unsigned int texture_height;
    float par;

    bool dirty;

    bool flip_vertically;
    bool flip_horizontally;
    bool rotate;

    ImVec2 last_pos;
    ImVec2 Last_extent;
};