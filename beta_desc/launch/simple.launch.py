import os
from typing import Text
from launch.substitution import Substitution
import xacro
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lxml import etree

default_namespace = 'beta_1'
packagename = 'beta_desc'

def generate_launch_description():

    robot_namespace = LaunchConfiguration('namespace', default=default_namespace)
    x_pos = LaunchConfiguration('x', default=0)
    y_pos= LaunchConfiguration('y', default=0)
    z_pos = LaunchConfiguration('z', default=0)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    
    share_path = get_package_share_directory(packagename)
    xacro_path = os.path.join(share_path, "urdf", "beta.xacro.xml")
    robot_description_config = Command(['xacro ', xacro_path, ' namespace:=',robot_namespace])
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
             )
             

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
        
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
        '-topic', 'robot_description',
        '-entity', 'beta',
        '-x', x_pos,
        '-y', y_pos,
        '-z', z_pos,
        ],
        output='screen')
                        



    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity,
        gazebo,
        DeclareLaunchArgument('use_sim_time',default_value='false  ',description='Use ros2_control if true'),
        DeclareLaunchArgument('namespace', default_value=robot_namespace,description='Robot namespace'),
    ])
