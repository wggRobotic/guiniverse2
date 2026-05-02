#pragma once

#include <memory>
#include <atomic>

#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
#include <imgui.h>

class RobotController
{
public:
    virtual void on_gui_frame(GLFWwindow* window) = 0;
};

class Guiniverse2
{
public:
    void run();

private:

    std::atomic<bool> running;
    std::shared_ptr<RobotController> robot_controller;
    int robot_index;
};