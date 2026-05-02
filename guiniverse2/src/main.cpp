#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <guiniverse2/guiniverse2.hpp>

#include <rclcpp/node.hpp>
#include <gst/gst.h>

int main(int argc, char **argv)
{
  if (!glfwInit()) return 1;
  glewInit();

  rclcpp::init(argc, argv);

  gst_init(NULL, NULL);

  Guiniverse2 gui;
  gui.run();

  rclcpp::shutdown();

  glfwTerminate();

  return 0;
}
