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
    void get_uv_base(ImVec2& pos, ImVec2& u_base, ImVec2& v_base);

    ImVec2 img_pos, img_u_base, img_v_base;
    
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