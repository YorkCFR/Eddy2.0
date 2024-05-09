import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import os

def generate_launch_description():

    # get the robot description
    urdf = os.path.join(get_package_share_directory('eddy_description'), 'eddy.urdf.xacro')
    robot_description = xacro.process_file(urdf , mappings={ 'robot_name' : 'Eddy2a' }).toxml()
    robot_state_publisher = Node(
             namespace = 'Eddy2a',
             package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             output='screen',
             parameters=[{'use_sim_time': True, 'robot_description': robot_description, 'frame_prefix' : 'Eddy2a/'}],
    )

    # get a description of Gazebo with the world required
    world = os.path.join(get_package_share_directory('eddy_gazebo'), 'stong_pond.world')
    gazebo_launch = IncludeLaunchDescription(
             PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),
                                           'launch', 'gazebo.launch.py'),
             ),
             launch_arguments = {'world': world}.items()
        )

    # spawn the robot
    spawn_robot = Node(
             package='gazebo_ros',
             executable='spawn_entity.py',
             name='urdf_spawner',
             output='screen',
             arguments=["-topic", "Eddy2a/robot_description",  "-entity",  "Eddy2a"])

    return LaunchDescription([
        gazebo_launch, 
        robot_state_publisher,
        spawn_robot,
    ])

