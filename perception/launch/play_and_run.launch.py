from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # ros2 bag play
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/rm/bag_files/rosbag2_2025_10_08-19_25_04', '--loop'],
            output='screen'
        ),

        # static transform publisher
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', '1', 'map', 'zed_left_camera_frame'],
            output='screen'
        ),

        # detection publisher python script
        ExecuteProcess(
            cmd=['python3', '/home/rm/detection_publisher.py'],
            output='screen'
        ),

        # depth2pose node from perception package
        ExecuteProcess(
            cmd=['ros2', 'run', 'perception', 'depth2pose_node'],
            output='screen'
        ),
    ])
