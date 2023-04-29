import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml



def generate_launch_description():

    bringup_dir = get_package_share_directory('beta_nav')
    beta_1_param = os.path.join(bringup_dir, 'params', 'beta_1_nav2.yaml')
    beta_2_param = os.path.join(bringup_dir, 'params', 'beta_2_nav2.yaml')
    beta_3_param = os.path.join(bringup_dir, 'params', 'beta_3_nav2.yaml')
    beta_4_param = os.path.join(bringup_dir, 'params', 'beta_4_nav2.yaml')


    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'yaml_filename': os.path.join(
            bringup_dir, 'map', 'wall_save.yaml'),
        'use_sim_time': 'true',
        'autostart': 'true'}

    beta_1_config = RewrittenYaml(
            source_file=beta_1_param,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)

    beta_2_config = RewrittenYaml(
            source_file=beta_2_param,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)

    beta_3_config = RewrittenYaml(
            source_file=beta_3_param,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)
    
    beta_4_config = RewrittenYaml(
            source_file=beta_4_param,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)
 

    lifecycle_nodes_nav = [
        'beta_1/controller_server',
        'beta_1/planner_server',
        'beta_1/bt_navigator',
        'beta_1/waypoint_follower',
        'beta_1/smoother_server',
        'beta_1/velocity_smoother',
        'beta_1/behavior_server',
        'beta_1/collision_monitor',

        'beta_2/controller_server',
        'beta_2/planner_server',
        'beta_2/bt_navigator',
        'beta_2/waypoint_follower',
        'beta_2/smoother_server',
        'beta_2/velocity_smoother',
        'beta_2/behavior_server',
         'beta_2/collision_monitor',

        'beta_3/controller_server',
        'beta_3/planner_server',
        'beta_3/bt_navigator',
        'beta_3/waypoint_follower',
        'beta_3/smoother_server',
        'beta_3/velocity_smoother',
        'beta_3/behavior_server',
         'beta_3/collision_monitor',

        'beta_4/controller_server',
        'beta_4/planner_server',
        'beta_4/bt_navigator',
        'beta_4/waypoint_follower',
        'beta_4/smoother_server',
        'beta_4/velocity_smoother',
        'beta_4/behavior_server',
         'beta_4/collision_monitor',
    ]

    return LaunchDescription([
    Node(
        namespace='beta_1',
        package='nav2_controller',
        name='controller_server',
        executable='controller_server',
        output='screen',
        parameters=[beta_1_config],
        remappings=[('/beta_1/cmd_vel', '/beta_1/cmd_vel_raw'),
                    ('/beta_1/local_costmap/beta_1/scan','/beta_1/scan')]),

    Node(
        namespace='beta_1',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[beta_1_config],
        remappings=[('/beta_1/map', '/map'),('/beta_1/global_costmap/beta_1/scan','/beta_1/scan')]),

    Node(
        namespace='beta_1',
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        emulate_tty=True,
        output='screen',
        parameters=[beta_1_config]),

    Node(
        namespace='beta_1',
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[beta_1_config]),
        

    Node(
        namespace='beta_1',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[beta_1_config]),

    Node(
        namespace='beta_1',
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[beta_1_config]),
    Node(
        namespace='beta_1',
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[beta_1_config]),
    Node(
        namespace='beta_1',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[beta_1_config]),


    Node(
        namespace='beta_2',
        package='nav2_controller',
        name='controller_server',
        executable='controller_server',
        output='screen',
        parameters=[beta_2_config],
        remappings=[('/beta_2/local_costmap/beta_2/scan','/beta_2/scan'),
                    ('/beta_2/cmd_vel', '/beta_2/cmd_vel_raw')]),

    Node(
        namespace='beta_2',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[beta_2_config],
        remappings=[('/beta_2/map', '/map'),('/beta_2/global_costmap/beta_2/scan','/beta_2/scan')]),

    Node(
        namespace='beta_2',
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        emulate_tty=True,
        output='screen',
        parameters=[beta_2_config]),
    
    Node(
        namespace='beta_2',
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[beta_2_config]),

    Node(
        namespace='beta_2',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[beta_2_config]),

    Node(
        namespace='beta_2',
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[beta_2_config]),
    Node(
        namespace='beta_2',
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[beta_2_config]),
    Node(
        namespace='beta_2',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[beta_2_config]),
    
    

    Node(
        namespace='beta_3',
        package='nav2_controller',
        name='controller_server',
        executable='controller_server',
        output='screen',
        parameters=[beta_3_config],
        remappings=[('/beta_3/local_costmap/beta_3/scan','/beta_3/scan'),
                    ('/beta_1/cmd_vel', '/beta_1/cmd_vel_raw')]),
    
    Node(
        namespace='beta_3',
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        emulate_tty=True,
        output='screen',
        parameters=[beta_3_config]),

    Node(
        namespace='beta_3',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[beta_3_config],
        remappings=[('/beta_3/map', '/map'),('/beta_3/global_costmap/beta_3/scan','/beta_3/scan')]),
    
    Node(
        namespace='beta_3',
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[beta_3_config]),

    Node(
        namespace='beta_3',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[beta_3_config]),

    Node(
        namespace='beta_3',
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[beta_3_config]),
    Node(
        namespace='beta_3',
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[beta_3_config]),
    Node(
        namespace='beta_3',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[beta_3_config]),

    
    Node(
        namespace='beta_4',
        package='nav2_controller',
        name='controller_server',
        executable='controller_server',
        output='screen',
        parameters=[beta_4_config],
        remappings=[('/beta_4/local_costmap/beta_4/scan','/beta_4/scan')
                     ,('/beta_4/cmd_vel', '/beta_4/cmd_vel_raw')]),


    Node(
        namespace='beta_4',
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[beta_4_config],
        remappings=[('/beta_4/map', '/map'),('/beta_4/global_costmap/beta_4/scan','/beta_4/scan')]),
    Node(
        namespace='beta_4',
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        emulate_tty=True,
        output='screen',
        parameters=[beta_4_config]),
    
    Node(
        namespace='beta_4',
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[beta_4_config]),

    Node(
        namespace='beta_4',
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[beta_4_config]),

    Node(
        namespace='beta_4',
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[beta_4_config]),
    Node(
        namespace='beta_4',
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[beta_4_config]),
    Node(
        namespace='beta_4',
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[beta_4_config]),



   Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes_nav},
                    {'bond_timeout':0.0}])
    ])


