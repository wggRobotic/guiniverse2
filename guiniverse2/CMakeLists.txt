cmake_minimum_required(VERSION 3.10)
project(guiniverse2)

cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##################################################
#---------------------ImGui----------------------#
##################################################

add_library(imgui STATIC
  lib/imgui/imgui.cpp
  lib/imgui/imgui_draw.cpp
  lib/imgui/imgui_tables.cpp
  lib/imgui/imgui_widgets.cpp
  lib/imgui/imgui_demo.cpp
  lib/imgui/backends/imgui_impl_opengl3.cpp
  lib/imgui/backends/imgui_impl_glfw.cpp
)
target_include_directories(imgui PUBLIC lib/imgui)

target_link_libraries(imgui
  OpenGL::GL
  ${GLEW_LIBRARIES}
  glfw
)

##################################################
#----------------------GLFW----------------------#
##################################################

add_subdirectory(lib/glfw)

##################################################
#------------------guiniverse2-------------------#
##################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quac_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(OpenGL REQUIRED) 
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLEW REQUIRED glew)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

add_executable(gui
  src/main.cpp
  src/guiniverse2/guiniverse2.cpp
  src/guiniverse2/joystick_input.cpp
  src/guiniverse2/video_display_gst.cpp
  src/guiniverse2/video_display_ros.cpp
  src/guiniverse2/imgui_utils.cpp
  src/guiniverse2/gui_image.cpp
  src/guiniverse2/detection_gallery.cpp
  src/quac/quac.cpp
  src/quac/cam_overlay.cpp
)

target_include_directories(gui PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  lib/quirc/lib

  include
)

target_link_libraries(gui
  glfw
  imgui
  OpenGL::GL
  ${GLEW_LIBRARIES}
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${OpenCV_LIBS}
)

ament_target_dependencies(gui
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  OpenCV
  quac_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

install(TARGETS gui
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
