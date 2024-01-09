import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	br_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['-0.5', '0', '0', '0', '0', '0', 'robot1/base_footprint', 'point1','10'],
        output = "screen"
    )
    
	listen_node = Node(
    	package='yahboomcar_multi',
    	executable='listenline',
    	output='screen'
    	)
    
	return LaunchDescription([
    	br_node,
    	#listen_node
    ])
    
    
