import os
from sys import argv
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    pkg_beta_desc = get_package_share_directory('beta_desc')
    #pkg_zalpha_nav = get_package_share_directory('zalpha_nav')

    ld = LaunchDescription()
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_beta_desc,
                         'launch', 'world.launch.py')           
        )))


    # Launch robots
    configs = [
        {'namespace': 'beta_1', 'x': '-4',
            'y': '1'},
        {'namespace': 'beta_2', 'x': '-4',
             'y': '-3'},
        {'namespace': 'beta_3', 'x': '5',
             'y': '3'},
        {'namespace': 'beta_4', 'x': '3',
             'y': '-4'},
     ]


    for config in configs:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_beta_desc,
                             'launch',
                             'spawn_launch.py')
            ),
            launch_arguments=config.items(),
        ))

    return ld
