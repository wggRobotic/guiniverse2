#include "imgui.h"
#include <mutex>
#include <guiniverse2/gui_image.hpp>
#include <GL/gl.h>

#define GLCALL(call) \
    do { \
        call; \
        GLenum err = glGetError(); \
        if (err != GL_NO_ERROR) { \
            fprintf(stderr, "OpenGL error in %s at %s:%d: %d\n", #call, __FILE__, __LINE__, err); \
        } \
    } while (0)

GuiImage::GuiImage(bool fv, bool fh, bool r)
{
    flip_vertically = fv; flip_horizontally = fh; rotate = r;

    GLCALL(glGenTextures(1, &gl_texture));
    GLCALL(glBindTexture(GL_TEXTURE_2D, gl_texture));

    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

    texture_width = 2;
    texture_height = 2;
    par = 1;
    dirty = false;
    unsigned int data[] = {0xff0000ff, 0xff00ff00, 0xffff0000, 0xff00ffff};
    
    GLCALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, texture_width, texture_height, 0, GL_BGR, GL_UNSIGNED_BYTE, data));                

    GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
}

GuiImage::~GuiImage()
{
    GLCALL(glDeleteTextures(1, &gl_texture));
}

ImVec2 GuiImage::uv_calc(ImVec2 uv)
{
    if (rotate) uv = ImVec2(1 - uv.y, uv.x);
    if (flip_horizontally) uv = ImVec2(1 - uv.x, uv.y);
    if (flip_vertically) uv = ImVec2(uv.x, 1 - uv.y);
    return uv;
}

void GuiImage::draw()
{
    ImGui::Checkbox("Flip vertically", &flip_vertically);
    ImGui::SameLine(0.0f, 10.0f);
    ImGui::Checkbox("Flip horizontally", &flip_horizontally);
    ImGui::SameLine(0.0f, 10.0f);
    ImGui::Checkbox("rotate", &rotate);

    {
        std::lock_guard<std::mutex> lock(mutex);
        
        if (dirty)
        {
            GLCALL(glBindTexture(GL_TEXTURE_2D, gl_texture));
            if (
                image.cols != texture_width || 
                image.rows != texture_height
            )
            {
                texture_width = image.cols;
                texture_height = image.rows;
                GLCALL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, texture_width, texture_height, 0, GL_BGR, GL_UNSIGNED_BYTE, image.data));                
            }
            else
            {
                GLCALL(glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texture_width, texture_height, GL_BGR, GL_UNSIGNED_BYTE, image.data));
            }
            GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
        }
    }

    ImVec2 widget_size = ImGui::GetContentRegionAvail();

    float image_aspect = (float)texture_width/ (float)texture_height * par;
    if (rotate) image_aspect = 1/image_aspect;

    float widget_aspect = widget_size.x / widget_size.y;

    ImVec2 display_size;

    if (widget_aspect > image_aspect)
        display_size = ImVec2(widget_size.y * image_aspect, widget_size.y);
    else 
        display_size = ImVec2(widget_size.x, widget_size.x / image_aspect);

    ImVec2 cursor = ImGui::GetCursorScreenPos();

    ImGui::GetWindowDrawList()->AddImageQuad(
        (ImTextureID)gl_texture,
        cursor,
        ImVec2(cursor.x + display_size.x, cursor.y),
        ImVec2(cursor.x + display_size.x, cursor.y + display_size.y),
        ImVec2(cursor.x, cursor.y + display_size.y),
        uv_calc({0, 0}), uv_calc({1, 0}), uv_calc({1, 1}), uv_calc({0, 1}),
        IM_COL32_WHITE
    );

    last_pos = cursor;
    Last_extent = display_size;
}

void GuiImage::draw_bounding_box(quac_interfaces::msg::BoundingBox& box)
{
    ImVec2 coords[4];
    for (int i = 0; i < 4; i++)
    {
        coords[i] = uv_calc({(float)box.corners[i].x, (float)box.corners[i].y});
        coords[i] = ImVec2(last_pos.x + coords[i].x * Last_extent.x, last_pos.y + coords[i].y * Last_extent.y);
    }

    for (int i = 0; i < 4; i++) ImGui::GetWindowDrawList()->AddLine(
        coords[i],
        coords[(i + 1) % 4],
        IM_COL32(255, 0, 0, 255),
        2.0f
    );
}

void GuiImage::set_image(cv::Mat& mat, float mat_par)
{
    std::lock_guard<std::mutex> lock(mutex);

    dirty = true;
    par = mat_par;
    image = mat;
}