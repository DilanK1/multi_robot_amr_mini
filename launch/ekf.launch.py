#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
from nav2_common.launch import RewrittenYaml
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

def render_xacro(context):
    namespace = LaunchConfiguration('namespace').perform(context)

    use_sim_time = LaunchConfiguration('use_sim_time')
    yaml_file = os.path.join(get_package_share_directory(
        'multi_robot_amr_mini'), 'config/ekf.yaml')
    multi_yaml = os.path.join(get_package_share_directory(
        'multi_robot_amr_mini'), 'config/multi_ekf.yaml')
    

    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)

    yaml_data = {namespace: data}
   
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
        'odom_frame': namespace+'/odom',
        'base_link_frame': namespace+'/base_footprint',
        'world_frame': namespace+'/odom'
    }
    
    configured_params = RewrittenYaml(
        source_file=multi_yaml,
        param_rewrites=param_substitutions)

    robot_localization_node = Node(
        namespace=namespace,
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[configured_params,
                    {'use_sim_time': use_sim_time}]
    )

    return [robot_localization_node]

def generate_launch_description():

    return LaunchDescription([

        OpaqueFunction(function=render_xacro)
    ])
