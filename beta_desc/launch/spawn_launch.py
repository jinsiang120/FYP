import os
from typing import Text
from launch.substitution import Substitution
import xacro
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lxml import etree

default_namespace = 'beta'
default_package_name = 'beta_desc'


class UrdfSubstitution(Substitution):
    def __init__(self, package_name, is_state_publisher=False) -> None:
        super().__init__()
        self.__xml = ""
        self.__package_name = package_name
        self.__is_state_publisher = is_state_publisher

    @property
    def expression(self) -> str:
        return self.__xml

    def perform(self, context: LaunchContext):
        self.__namespace = context.launch_configurations["namespace"]
        share_path = get_package_share_directory(self.__package_name)
        # Generate xacro file
        xacro_path = os.path.join(
            share_path, "urdf", 'beta.xacro.xml')

        # Generate xacro
        mappings = {
            'namespace':self.__namespace,
        }

        self.__xml = xacro.process(xacro_path, mappings=mappings)
        self.update_URDF()
        return str(self.__xml)

    def update_URDF(self):
        # URDF namepsace updateg
        xml = self.__xml
        urdf = ET.fromstring(xml)

def generate_launch_description():

    robot_namespace = LaunchConfiguration('namespace', default=default_namespace)
    x = LaunchConfiguration('x', default=0)
    y = LaunchConfiguration('y', default=0)
    z = LaunchConfiguration('z', default=0.2)

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value=x,
                              description='X position'),
        DeclareLaunchArgument('y', default_value=y,
                              description='Y position'),
        DeclareLaunchArgument('z', default_value=z,
                              description='Z position'),
        Node(
            package=default_package_name,
            executable='spawn_beta',
            name='spawn_beta',
            namespace=robot_namespace,
            output='screen',
            parameters=[{
                'name': robot_namespace,
                'namespace': robot_namespace,
                'xml': UrdfSubstitution(default_package_name),
                'x': x,
                'y': y,
                'z': z,
            }]),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_namespace,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': UrdfSubstitution(default_package_name, True)
            }]),
    ])


