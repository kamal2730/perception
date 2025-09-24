# ROS BOG
```
unzip rosbag2_2025_09_24-20_20_15.zip
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map zed_left_camera_frame
ros2 bag play rosbag2_2025_09_24-20_20_15 --loop
```
![ros_bag](../assets/ros_bag.png)