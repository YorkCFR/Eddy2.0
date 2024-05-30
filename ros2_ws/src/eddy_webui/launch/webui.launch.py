import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    root = "."
    port = 8080
    for arg in sys.argv: # there must be a better way...
        print(arg)
        if arg.startswith('root:='):
           print(arg.split('root:=', 1)[1])
           root = arg.split('root:=', 1)[1]
        elif arg.startswith('port:='):
           print(arg.split('port:=', 1)[1])
#           port = int(arg.split('port:=', 1)[1])
        elif ':=' in arg:
           print(f"Unknown argument in {arg}")
           sys.exit(0)

    return LaunchDescription([
        Node(
             package='eddy_webui',
             executable='web_ui',
             name='web_ui',
             output='screen',
             parameters=[{'root': root, 'port': port}]),
    ])

