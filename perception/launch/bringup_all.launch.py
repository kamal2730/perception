from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory 

import os

def generate_launch_description():
    # ZED wrapper launch file
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ]),
        launch_arguments={'camera_model': 'zed2i'}.items()
    )

    # RGBD Server Node
    rgbd_server = Node(
        package='rgbd',
        executable='rgbd_server',
        name='rgbd_server',
        output='screen',
    )

    # Detection Visualizer
    dv_node = Node(
        package='rgbd',
        executable='dv',
        name='detection_visualizer',
        output='screen'
    )

    # Cluster Merger Node
    cluster_merger = Node(
        package='perception',
        executable='cluster_merger_node',
        name='cluster_merger_node',
        output='screen'
    )

    # Depth2Pose Node
    depth2pose = Node(
        package='perception',
        executable='depth2pose_node',
        name='depth2pose_node',
        output='screen'
    )

    return LaunchDescription([
        zed_launch,
        rgbd_server,
        dv_node,
        cluster_merger,
        depth2pose
    ])
