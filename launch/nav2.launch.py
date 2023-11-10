#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import yaml
from launch.actions import OpaqueFunction


def render_xacro(context):

    namespace = LaunchConfiguration('namespace').perform(context)

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')    
    params_file = os.path.join(get_package_share_directory('multi_robot_amr_mini'), 'config/nav2_params.yaml')
    multi_yaml = os.path.join(get_package_share_directory('multi_robot_amr_mini'), 'config/multi_nav2_params.yaml')
        
    with open(params_file, 'r') as file:
        data = yaml.safe_load(file)

    yaml_data = {namespace: data}
    yaml_data[namespace]['bt_navigator']['ros__parameters']['global_frame'] = "map"
    yaml_data[namespace]['global_costmap']['global_costmap']['ros__parameters']['global_frame'] =  "map"
    yaml_data[namespace]['global_costmap']['global_costmap']['ros__parameters']['map_topic'] =  "map"
    yaml_data[namespace]['local_costmap']['local_costmap']['ros__parameters']['global_frame'] = namespace+"/odom"
    yaml_data[namespace]['local_costmap']['local_costmap']['ros__parameters']['robot_base_frame'] = namespace +"/base_footprint"
    yaml_data[namespace]['global_costmap']['global_costmap']['ros__parameters']['robot_base_frame'] = namespace +"/base_footprint"

    if os.path.exists(multi_yaml):

        with open(multi_yaml, 'r') as file:
            existing_data = yaml.safe_load(file)

            existing_data.update(yaml_data)
        with open(multi_yaml, 'w') as file:
            yaml.dump(existing_data, file, default_flow_style=False)

    else:

        with open(multi_yaml, 'w') as file:
            yaml.dump(yaml_data, file, default_flow_style=False)


    param_substitutions = {
        'robot_base_frame': namespace+'/base_footprint',
        'odom_topic': namespace+'/odom',
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local,

    }    

    lifecycle_nodes = [namespace+'/controller_server',
                       namespace+'/planner_server',
                       namespace+'/behavior_server',
                       namespace+'/bt_navigator',
                       namespace+'/waypoint_follower'
                       #    'collision_monitor'
                       ]

    remappings = [('/tf', '/tf'),
                  ('/tf_static', '/tf_static'),
                  ('cmd_vel' ,'cmd_vel_nav'),
                  ('map', '/map'),
                  ('/front_lidar_amr_mini_laser' ,"/"+ namespace +'/front_lidar_amr_mini_laser' ),
                  ('/back_lidar_amr_mini_laser' ,"/"+namespace+'/back_lidar_amr_mini_laser' )

                  ]

    # Create our own temporary YAML files that include substitutions

    configured_params = RewrittenYaml(
        source_file=multi_yaml,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    controller_server = Node(
        namespace=namespace,
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    planner_server = Node(
        namespace=namespace,
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    behavior_server = Node(
        namespace=namespace,
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    bt_navigator = Node(
        namespace=namespace,
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    waypoint_follower = Node(
        namespace=namespace,
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    # collision_monitor = Node(
    #     namespace=namespace,
    #     package='nav2_collision_monitor',
    #     executable='collision_monitor',
    #     name='collision_monitor',
    #     output='screen',
    #     parameters=[configured_params])

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'bond_timeout': 0.0},
                    {'node_names': lifecycle_nodes}])
    

    return [controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        # collision_monitor,
        lifecycle_manager]

def generate_launch_description():
  
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='true',
            description='Whether to set the map subscriber QoS to transient local'),

        OpaqueFunction(function=render_xacro),

    ])
