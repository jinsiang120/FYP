import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    share_path = get_package_share_directory("beta_mapping")
    start_async_slam_toolbox_node = Node(
        parameters=[
          os.path.join(share_path, 'mapper_params_online_async.yaml'),
          {'use_sim_time': True}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()


    ld.add_action(start_async_slam_toolbox_node)

    return ld
