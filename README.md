# guiniverse2
Second iteration of c++ gui for controlling our robots over ros2

## build
look into the dockerfile for dependencies. For the ros2 packages you might need to change the distribution in the install commands from humble to your distribution (e.g. jazzy).

make sure you pulled the git submodules
```git submodule update --init --recursive```

also make sure to have the [quac_interfaces](https://github.com/wggRobotic/quac-interfaces) package installed

## Interfaces for quac

#### Driving:

- Publisher `/quac/cmd_vel_pilot : geometry_msgs/msg/TwistStamped`
linear and angular velocities 

#### Arm:

- Publisher `/quac/ee_pose : geometry_msgs/msg/Pose2D`
target rm target pose in meters and radians

- Publisher `/quac/gripper_width : std_msgs/msg/Float64`
target width between the left and right gripperclams in meters

- Subscription `/quac/joint_states : sensor_msgs/msg/JointState`


#### Map: 
- Subscription `/quac/map : nav_msgs/msg/OccupancyGrid`
map to display inside the gui

#### Cameras:

- Publisher `/quac/video_target_ip : std_msgs/msg/String`
ip that the camera streams should stream to

- TF-Listener on `/quac/tf` and `/quac/tf_static` for camera positions to draw path lines

- Subscription `/quac/<camera_name>/info : sensor_msgs::msg::CameraInfo`
camera info for instrinsics to draw path lines
`<camera_name>` can be `camera_front`, `camera_gripper`, `camera_back`

- Subscription `/quac/<camera_name>/<object_type>/bounding_boxes : quac_interfaces::msg::BoundingBoxArray`
bounding boxes overlay in gui
`<object_type>` can be `qrcodes`, `hazmat_signs`, `landolt_cs`, `paintrollers`

- the front, back, gripper, left and right cameras themself are transmitted over a gstreamer udp rtp h.264 stream.

- Subscription `/quac/thermal_image/compressed : sensor_msgs/msg/CompressedImage` 
thermal image subscriber

#### Detected objects

- Subscription `/quac/<object_type>/json : std_msgs/msg/String`
json file with all the detected objects positions and types

- Subscription `/quac/<object_type>/images : sensor_msgs/msg/CompressedImage`
best images of detected objects with marking and name

#### Imu

- Subscription `/quac/camera_front/imu : sensor_msgs/msg/Imu`
Data from the imu of the Oak D Lite

- Subscription `/quac/magnetic_field : sensor_msgs/msg/MagneticField`
Measurements from the magnetometer