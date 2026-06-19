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
            fflush(stderr); \
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

    ImVec2 uv[4] =
    {
        ImVec2(0.f, rotate ? 1.f : 0.f), 
        ImVec2(rotate ? 0.f : 1.f, 0.f), 
        ImVec2(1.f, rotate ? 0.f : 1.f),
        ImVec2(rotate ? 1.f : 0.f, 1.f), 
    };

    img_pos = ImVec2(rotate ? display_size.x : 0.f, 0.f);
    img_u_base = (rotate ? ImVec2(0.f, display_size.y) : ImVec2(display_size.x, 0.f));
    img_v_base = (rotate ? ImVec2(-display_size.x, 0.f) : ImVec2(0.f, display_size.y));

    if (flip_horizontally)
    {
        ImVec2 temp = uv[0];
        uv[0] = uv[1];
        uv[1] = temp;

        temp = uv[2];
        uv[2] = uv[3];
        uv[3] = temp;

        img_pos = ImVec2(display_size.x - img_pos.x, img_pos.y);
        img_u_base = ImVec2(-img_u_base.x, img_u_base.y);
        img_v_base = ImVec2(-img_v_base.x, img_v_base.y);
    }

    if (flip_vertically)
    {
        ImVec2 temp = uv[0];
        uv[0] = uv[3];
        uv[3] = temp;

        temp = uv[1];
        uv[1] = uv[2];
        uv[2] = temp;

        img_pos = ImVec2(img_pos.x, display_size.y - img_pos.y);
        img_u_base = ImVec2(img_u_base.x, -img_u_base.y);
        img_v_base = ImVec2(img_v_base.x, -img_v_base.y);
    }

    img_pos = ImVec2(img_pos.x + cursor.x, img_pos.y + cursor.y);
    
    ImGui::GetWindowDrawList()->AddImageQuad(
        (ImTextureID)gl_texture,
        cursor,
        ImVec2(cursor.x + display_size.x, cursor.y),
        ImVec2(cursor.x + display_size.x, cursor.y + display_size.y),
        ImVec2(cursor.x, cursor.y + display_size.y),
        uv[0], uv[1], uv[2], uv[3],
        IM_COL32_WHITE
    );
}

void GuiImage::set_image(cv::Mat& mat, float mat_par)
{
    std::lock_guard<std::mutex> lock(mutex);

    dirty = true;
    par = mat_par;
    image = mat;
}

void GuiImage::get_uv_base(ImVec2& pos, ImVec2& u_base, ImVec2& v_base)
{
    pos = img_pos;
    u_base = img_u_base;
    v_base = img_v_base;
}