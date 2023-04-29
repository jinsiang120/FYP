import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('beta_nav')
    lifecycle_nodes = ['map_server', 'beta_1/amcl','beta_2/amcl','beta_3/amcl','beta_4/amcl']


    beta_1_config = os.path.join(bringup_dir, 'params', 'beta_1_nav2.yaml')
    beta_2_config = os.path.join(bringup_dir, 'params', 'beta_2_nav2.yaml')
    beta_3_config = os.path.join(bringup_dir, 'params', 'beta_3_nav2.yaml')
    beta_4_config = os.path.join(bringup_dir, 'params', 'beta_4_nav2.yaml')
    map_file = os.path.join(bringup_dir, 'map', 'wall_save.yaml')


    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'topic_name':"map"},
                        {'frame_id':"map"},
                        {'yaml_filename':map_file}]
        ),
            
        Node(
            namespace='beta_1',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[beta_1_config],
        ),

        Node(
             namespace='beta_2',
             package='nav2_amcl',
             executable='amcl',
             name='amcl',
             output='screen',
             parameters=[beta_2_config],
        ),

        Node(
            namespace='beta_3',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[beta_3_config],
        ),

        Node(
             namespace='beta_4',
             package='nav2_amcl',
             executable='amcl',
             name='amcl',
             output='screen',
             parameters=[beta_4_config],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes},
                        {'bond_timeout':0.0},]
        ),
    ])
    


