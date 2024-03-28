import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # get the robot description and customize it
    urdf = os.path.join(get_package_share_directory('eddy_description'), 'eddy.urdf.xacro')
    print(urdf)
    robot_description = xacro.process_file(urdf , mappings={
        'robot_name' : 'Eddy2a',
        'gps_latitude_origin' : '43.77067122214868',
        'gps_longitude_origin' : '-79.50699748853243'
        }).toxml()
    print(robot_description)

    robot_state_publisher = Node(
             package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             output='screen',
             parameters=[{'use_sim_time': True, 'robot_description': robot_description, 'frame_prefix' : 'Eddy2a/'}],
    )

    return LaunchDescription([
        IncludeLaunchDescription(
             PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),
                                           'launch', 'gazebo.launch.py'),
             )
        ),
        robot_state_publisher,
        Node(
             package='gazebo_ros',
             executable='spawn_entity.py',
             name='urdf_spawner',
             output='screen',
             arguments=["-topic", "/robot_description",  "-entity",  "Eddy2a"]),
    ])

