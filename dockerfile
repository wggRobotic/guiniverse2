FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt update
RUN apt install -y ros-humble-cv-bridge ros-humble-image-transport ros-humble-compressed-image-transport
RUN apt install -y ros-humble-rmw-cyclonedds-cpp

RUN apt install -y \
  libopencv-dev \
  libglew-dev \
  libglfw3-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  gstreamer1.0-libav \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  pkg-config

RUN apt install -y \
  build-essential \
  cmake \
  git

RUN apt install -y \
  libx11-dev \
  libxrandr-dev \
  libxinerama-dev \
  libxcursor-dev \
  libxi-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev

RUN apt install -y \
  libwayland-dev \
  libwayland-client0 \
  libwayland-cursor0 \
  libwayland-egl1-mesa \
  libxkbcommon-dev

RUN apt install iproute2 -y
RUN apt update

WORKDIR /ros2_ws

RUN git clone https://github.com/wggRobotic/quac-interfaces.git src/quac-interfaces
RUN . /opt/ros/humble/setup.bash && colcon build

COPY ./guiniverse2 src/guiniverse2
RUN . /opt/ros/humble/setup.bash && . install/setup.bash && colcon build
