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

default_namespace = 'beta1'
packagename = 'beta_desc'


class UrdfSubstitution(Substitution):
    def __init__(self, xml: str, is_state_publisher = False) -> None:
        super().__init__()
        self.__xml = xml
        self.__is_state_publisher = is_state_publisher

    @property
    def expression(self) -> Text:
        return self.__xml

    def perform(self, context: LaunchContext) -> Text:
        self.__namespace = context.launch_configurations["namespace"]
        self.update_URDF()
        return str(self.__xml)

    def update_URDF(self):
        # URDF namepsace update
        namespace = self.__namespace
        xml = self.__xml

        urdf = ET.ElementTree(ET.fromstring(xml))


        for tag in ['link', 'joint']:
            for element in urdf.iter(tag):
                element.set('name', namespace + '/' + element.get('name'))
        for tag in ['parent', 'child']:
            for element in urdf.iter(tag):
                element.set('link', namespace + '/' + element.get('link'))
        for tag in ['gazebo']:
            for element in urdf.iter(tag):
                if(element.get('reference')):
                    element.set('reference', namespace +
                                '/' + element.get('reference'))          
        for tag in ['left_joint', 'right_joint', 'joint_name',"robot_base_frame", "frame_name", "odometry_frame"]:
            for element in urdf.iter(tag):
                element.text = str(namespace + '/' + element.text)
        for element in urdf.iter("namespace"):
            element.text = str('/' + namespace)
        if(self.__is_state_publisher):
            for tag in ['mesh']:
                for element in urdf.iter(tag):
                    element.set('filename', 'file://' +
                                element.get('filename'))
        self.__xml = ET.tostring(urdf.getroot(), encoding='unicode', method='xml')
        return self.__xml



def generate_launch_description():
    share_path = get_package_share_directory(packagename)
    xacro_path = os.path.join(share_path, "urdf", "beta.xacro.xml")
    

    robot_namespace = LaunchConfiguration('namespace', default=default_namespace)

    
    x = LaunchConfiguration('x', default=0)
    y = LaunchConfiguration('y', default=0)
    z = LaunchConfiguration('z', default=0.2)

    xml = xacro.process(xacro_path,  mappings={'namespace':robot_namespace}).toxml()

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    return LaunchDescription([
        gazebo,
        DeclareLaunchArgument('namespace', default_value=robot_namespace,
                              description='Robot namespace'),
        DeclareLaunchArgument('x', default_value=x,
                              description='X position'),
        DeclareLaunchArgument('y', default_value=y,
                              description='Y position'),
        DeclareLaunchArgument('z', default_value=z,
                              description='Z position'),
        Node(
            package=packagename,
            executable='spawn_beta',
            name='spawn_beta',
            namespace=robot_namespace,
            output='screen',
            parameters=[{
                'name': robot_namespace,
                'namespace': robot_namespace,
                'xml': UrdfSubstitution(xml),
                'x': x,
                'y': y,
                'z': z,
            }]),
        Node(
            package='robot_state_publisher',
            namespace=robot_namespace,
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': UrdfSubstitution(xml, True)
            }])
    ])
