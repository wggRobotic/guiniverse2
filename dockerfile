FROM ros:humble

SHELL ["/bin/sh", "-c"]

RUN apt-get update
RUN apt-get install -y \
  build-essential \
  cmake \
  git \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-compressed-image-transport \
  ros-humble-rmw-cyclonedds-cpp \
  libopencv-dev \
  libglew-dev \
  libglfw3-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-libav \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  pkg-config \
  libx11-dev \
  libxcursor-dev \
  libxi-dev \
  libxinerama-dev \
  libxkbcommon-dev \
  libxrandr-dev \
  libwayland-dev \
  libwayland-client0 \
  libwayland-cursor0 \
  libwayland-egl1-mesa \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  iproute2

WORKDIR /ros2_ws

COPY ./guiniverse2 src/guiniverse2
COPY ./guiniverse2_launch src/guiniverse2_launch

RUN git clone https://github.com/wggRobotic/quac-interfaces.git src/quac-interfaces

RUN . /opt/ros/humble/setup.sh && colcon build
