#include "quac/cam_overlay.hpp"
#include "imgui.h"
#include "quac_interfaces/msg/bounding_box.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <rclcpp/logging.hpp>

constexpr float near_plane = 0.001f;

bounding_box_topic_manager::bounding_box_topic_manager(const std::string& topic, rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group)
{
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;

    subscriber = node->create_subscription<quac_interfaces::msg::BoundingBoxArray>(
        topic, 
        rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](const quac_interfaces::msg::BoundingBoxArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex);
            box_array = *msg;
        },
        options
    );

    show = true;
}

void bounding_box_topic_manager::draw(GuiImage* image, ImU32 color)
{
    if (!show) return;

    for (quac_interfaces::msg::BoundingBox& box : box_array.boxes)
    {
        ImVec2 pos, u_base, v_base;
        image->get_uv_base(pos, u_base, v_base);

        ImVec2 coords[4];
        for (int i = 0; i < 4; i++)
            coords[i] = ImVec2(
                (float)box.corners[i].x * u_base.x + (float)box.corners[i].y * v_base.x + pos.x, 
                (float)box.corners[i].x * u_base.y + (float)box.corners[i].y * v_base.y + pos.y
            );

        for (int i = 0; i < 4; i++) ImGui::GetForegroundDrawList()->AddLine(
            coords[i],
            coords[(i + 1) % 4],
            color,
            2.0f
        );
    }
   
}

CamOverlay::CamOverlay(
    const std::string& cam_panel_name,
    GuiImage* img, 
    const std::string& cam_name,
    rclcpp::Node::SharedPtr node,
    rclcpp::CallbackGroup::SharedPtr group,
    tf2_ros::Buffer* buffer
) :
node(node),
panel_name(cam_panel_name + " - settings"),
image(img),
qrcode_manager(cam_name + "/qrcodes/bounding_boxes", node, group),
hazmat_manager(cam_name + "/hazmat_signs/bounding_boxes", node, group),
landolt_manager(cam_name + "/landolt_cs/bounding_boxes", node, group),
paintroller_manager(cam_name + "/paintrollers/bounding_boxes", node, group),
cam_frame(cam_name + "_color_optical_frame"),
tf_buffer(buffer)
{
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;

    cam_info_subscriber = node->create_subscription<sensor_msgs::msg::CameraInfo>(
        cam_name+"/info",
        rclcpp::QoS(2).best_effort(),
        [this](sensor_msgs::msg::CameraInfo::SharedPtr msg)
        {
            cam_int_devided.fx = msg->k[0] / (float)msg->width;
            cam_int_devided.cx = msg->k[2] / (float)msg->width;
            cam_int_devided.fy = msg->k[4] / (float)msg->height;
            cam_int_devided.cy = msg->k[5] / (float)msg->height;
        },
        options
    );

    dynamic_robot_path = false;
    dynamic_path_seconds = 5.f;
    fixed_path_length = 2.f;

    cam_int_devided.fx = 602.875f / 640.f;
    cam_int_devided.fy = 601.773f / 480.f;
    cam_int_devided.cx = 312.289f / 640.f;
    cam_int_devided.cy = 246.923f / 480.f;
}

bool CamOverlay::project_line_points(
    const tf2::Vector3& a_world,
    const tf2::Vector3& b_world,
    ImVec2& out_a,
    ImVec2& out_b
)
{
    constexpr float eps = 1e-6f;

    // Transform into camera space
    tf2::Vector3 a = base_to_cam * a_world;
    tf2::Vector3 b = base_to_cam * b_world;

    bool a_front = a.z() > near_plane;
    bool b_front = b.z() > near_plane;

    // Fully behind camera
    if (!a_front && !b_front)
        return false;

    // Clip against near plane
    if (a_front != b_front)
    {
        float dz = b.z() - a.z();

        // Degenerate / nearly parallel to near plane
        if (std::abs(dz) < eps)
            return false;

        float t = (near_plane - a.z()) / dz;

        // Clamp for numerical safety
        t = std::clamp(t, 0.0f, 1.0f);

        tf2::Vector3 p = a + (b - a) * t;

        if (!a_front)
            a = p;
        else
            b = p;
    }

    // Prevent division by tiny Z
    if (a.z() < near_plane || b.z() < near_plane)
        return false;

    // Camera intrinsics -> normalized UV
    auto projectUV = [&](const tf2::Vector3& p) -> ImVec2
    {
        float inv_z = 1.0f / p.z();

        float px = cam_int_devided.fx * (p.x() * inv_z) + cam_int_devided.cx;
        float py = cam_int_devided.fy * (p.y() * inv_z) + cam_int_devided.cy;

        return ImVec2(
            px,
            py
        );
    };

    ImVec2 uv0 = projectUV(a);
    ImVec2 uv1 = projectUV(b);

    // Liang–Barsky clip against UV box [0,1]^2
    float t0 = 0.0f;
    float t1 = 1.0f;

    float dx = uv1.x - uv0.x;
    float dy = uv1.y - uv0.y;

    auto clip = [&](float p, float q) -> bool
    {
        if (std::abs(p) < eps)
            return q >= 0.0f;

        float r = q / p;

        if (p < 0.0f)
        {
            if (r > t1)
                return false;

            if (r > t0)
                t0 = r;
        }
        else
        {
            if (r < t0)
                return false;

            if (r < t1)
                t1 = r;
        }

        return true;
    };

    if (
        !clip(-dx, uv0.x)       || // left
        !clip( dx, 1.0f-uv0.x) || // right
        !clip(-dy, uv0.y)       || // top
        !clip( dy, 1.0f-uv0.y)    // bottom
    )
    {
        return false;
    }

    ImVec2 c0(
        uv0.x + t0 * dx,
        uv0.y + t0 * dy
    );

    ImVec2 c1(
        uv0.x + t1 * dx,
        uv0.y + t1 * dy
    );

    // Reject degenerate projected lines
    {
        float ddx = c1.x - c0.x;
        float ddy = c1.y - c0.y;

        if ((ddx * ddx + ddy * ddy) < eps)
            return false;
    }

    // UV -> screen space
    auto uvToScreen = [&](const ImVec2& uv) -> ImVec2
    {
        return ImVec2(
            uv.x * image->img_u_base.x +
            uv.y * image->img_v_base.x +
            image->img_pos.x,

            uv.x * image->img_u_base.y +
            uv.y * image->img_v_base.y +
            image->img_pos.y
        );
    };

    out_a = uvToScreen(c0);
    out_b = uvToScreen(c1);

    return true;
}

void CamOverlay::draw(float v_x, float omega_z)
{
    qrcode_manager.draw(image, IM_COL32(255, 0, 0, 255));
    hazmat_manager.draw(image, IM_COL32(255, 0, 0, 255));
    landolt_manager.draw(image, IM_COL32(255, 0, 0, 255));
    paintroller_manager.draw(image, IM_COL32(255, 0, 0, 255));

    try {
        geometry_msgs::msg::TransformStamped base_to_cam_msg = tf_buffer->lookupTransform(
            cam_frame,
            "base_link",
            tf2::TimePointZero
        );

        tf2::fromMsg(base_to_cam_msg.transform, base_to_cam);
    }
    catch (const tf2::TransformException &ex) { return; }

#define WHEEL_X 0.114
#define WHEEL_Y 0.113

    ImVec2 wheel_positions[4] = {
        ImVec2(WHEEL_X, WHEEL_Y),
        ImVec2(WHEEL_X, -WHEEL_Y),
        ImVec2(-WHEEL_X, WHEEL_Y),
        ImVec2(-WHEEL_X, -WHEEL_Y),
    };

    for (ImVec2 pos : wheel_positions)
    {
        tf2::Vector3 pos_center_global(pos.x, pos.y, 0.0);
        tf2::Vector3 pos_center_cam = base_to_cam * pos_center_global;

        if (pos_center_cam.z() > near_plane)
        {
            ImVec2 center_uv(
                (cam_int_devided.fx * (pos_center_cam.x() / pos_center_cam.z()) + cam_int_devided.cx),
                (cam_int_devided.fy * (pos_center_cam.y() / pos_center_cam.z()) + cam_int_devided.cy)
            );

            if (center_uv.x >= 0.f && center_uv.x <= 1.f && center_uv.y >= 0.f && center_uv.y <= 1.f)
            {
                ImGui::GetForegroundDrawList()->AddCircleFilled(
                    ImVec2(
                        center_uv.x * image->img_u_base.x +
                        center_uv.y * image->img_v_base.x +
                        image->img_pos.x,

                        center_uv.x * image->img_u_base.y +
                        center_uv.y * image->img_v_base.y +
                        image->img_pos.y
                    ),
                    10.f,
                    IM_COL32(255, 255, 255, 255)
                );
            }
        }

        if (dynamic_robot_path && dynamic_path_seconds == 0.0) continue;;
        if (fixed_path_length == 0.0) continue;
        if (v_x == 0 && omega_z == 0) continue;

        if(omega_z == 0)
        {
            tf2::Vector3 a_world(0.0 + pos.x, 0.0 + pos.y, 0.0);
            tf2::Vector3 b_world((dynamic_robot_path ? dynamic_path_seconds * v_x : v_x / std::abs(v_x) * fixed_path_length) + pos.x, 0.0 + pos.y, 0.0);

            ImVec2 a_uv, b_uv;
            if (project_line_points(a_world, b_world, a_uv, b_uv))
            {
                ImGui::GetForegroundDrawList()->AddLine(
                    a_uv,
                    b_uv,
                    IM_COL32(255, 0, 0, 255),
                    5.0f
                );   
            }
        }
        else
        {
            float vel = sqrtf((v_x-omega_z*pos.y)*(v_x-omega_z*pos.y) + pos.x*pos.x*omega_z*omega_z);
            float time = dynamic_path_seconds;
            if (!dynamic_robot_path) time = fixed_path_length / vel;
            
            int seg_count = std::max(std::abs(omega_z) * time * 40.f, std::abs(vel) * time / 0.02f);

            std::vector<tf2::Vector3> points_global;
            points_global.resize(seg_count+1);

            for (int i = 0; i <= seg_count; i++)
            {
                float theta = time * (float)i / (float)seg_count * omega_z;

                points_global[i] = tf2::Vector3(
                    (v_x/omega_z-pos.y)*sinf(theta)+pos.x*cosf(theta),
                    -((v_x/omega_z-pos.y)*cosf(theta)-pos.x*sinf(theta)-v_x/omega_z),
                    0.f
                );
            }

            for (int i = 0; i < seg_count; i++)
            {
                ImVec2 a_uv, b_uv;
                if (project_line_points(points_global[i], points_global[i+1], a_uv, b_uv))
                {
                    ImGui::GetForegroundDrawList()->AddLine(
                        a_uv,
                        b_uv,
                        IM_COL32(255, 0, 0, 255),
                        5.0f
                    );   
                }
            }
        }
    }

}

void CamOverlay::settings_panel()
{
    if (ImGui::Begin(panel_name.c_str()))
    {
        ImGui::Checkbox("qrcode boxes", &qrcode_manager.show);
        ImGui::Checkbox("hazmat sign boxes", &hazmat_manager.show);
        ImGui::Checkbox("landolt c boxes", &landolt_manager.show);
        ImGui::Checkbox("paintroller boxes", &paintroller_manager.show);
        ImGui::Checkbox("robot path dynamic", &dynamic_robot_path);
        ImGui::SliderFloat("Dynamic path seconds", &dynamic_path_seconds, 0.0f, 10.f);
        ImGui::SliderFloat("Fixed path length", &fixed_path_length, 0.0f, 10.f);
    }
    ImGui::End();
}