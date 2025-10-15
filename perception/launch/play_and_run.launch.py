from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # # ros2 bag play
        # ExecuteProcess(
        #     cmd=['ros2', 'bag', 'play', '/home/rm/bag_files/rosbag2_2025_10_08-19_25_04', '--loop'],
        #     output='screen'
        # ),

        # # static transform publisher
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', '1', 'map', 'zed_left_camera_frame'],
        #     output='screen'
        # ),

        # # detection publisher python script
        # ExecuteProcess(
        #     cmd=['python3', '/home/rm/detection_publisher.py'],
        #     output='screen'
        # ),

       ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-r', '1', '/test_detections',
                'custom_interfaces/msg/RgbDetection',
                '{image: {width: 640, height: 360, encoding: "rgb8"}, objects: [{name: "TARGET_OBJECT", x: 327.0, y: 24.0, width: 95.0, height: 115.0, probability: 0.99}]}'
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'perception', 'detection_visualizer_node'],
            output='screen'
        ),
        
        # depth2pose node from perception package
        ExecuteProcess(
            cmd=['ros2', 'run', 'perception', 'depth2pose_node'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'perception', 'cluster_merger_node'],
            output='screen'
        ),
    ])
