import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # map GPS coordinates to ROS coordinates
    navsat_node = Node(
             package='robot_localization',
             executable='navsat_transform_node',
             name='navsat_transform_node',
             output='screen',
             namespace='Eddy2a',
             parameters=[{'use_sim_time': True}, 
                         {'yaw_offset' : 0.0},                    # yaw is aligned with robot
                         {'magnetic_declanation_radians' : 0.0},  # magnetic declarantion for your location (see www.ngdc.noaa.gov.geomag-web)
                         {'broadcast_utm_transform_as_parent_frame' : True},  # put utm as the root
                         {'broadcast_cartesian_transform' : True},      # and broadcast it
                         {'baselink_frame_id' : 'Eddy2a/base_footprint'}, 
                         {'wait_for_datum': False},               # dont wait for datum
                         {'use_odometry_yaw': False},             # use the imu
                         {'zero_altitude' : True}],               # on the ground
             remappings=[('gps/fix', '/Eddy2a/gps'),
                         ('odometry/filtered', '/Eddy2a/odom'),
#                         ('imu', '/Edd2a/imu')
                        ]

    )

    return LaunchDescription([
        navsat_node,
    ])

