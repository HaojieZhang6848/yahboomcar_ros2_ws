from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    web_video_node = Node(
        package='web_video_server',
        executable='web_video_server',
        output='screen'
    )

    return LaunchDescription([
        web_video_node
    ])
