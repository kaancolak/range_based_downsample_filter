from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_prefix = get_package_share_directory('range_based_downsample_filter')
    
    config_file = os.path.join(pkg_prefix, 'config/range_based_downsample_filter.param.yaml')

    return LaunchDescription([
        Node(
            package='range_based_downsample_filter',
            executable='range_based_downsample_filter_node',
            name='range_based_downsample_filter',
            parameters=[config_file],
            remappings=[
                ('input', '/perception/pointcloud_densifier/pointcloud'),
                ('output', '/perception/pointcloud_densifier/pointcloud/downsampled')
            ],
        )
    ])