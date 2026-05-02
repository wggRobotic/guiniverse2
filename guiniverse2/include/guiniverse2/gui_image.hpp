#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <imgui.h>

class GuiImage
{
public:
    GuiImage() = delete;
    GuiImage(bool fv, bool fh, bool r);
    ~GuiImage();

    void draw();
    void set_image(cv::Mat& mat, float mat_par);

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
};