import os
import launch
from launch_ros.actions import Node
import launch_ros
from launch.substitutions import LaunchConfiguration
import yaml
from launch.actions import OpaqueFunction


def render_xacro(context):

    namespace = LaunchConfiguration('namespace').perform(context)

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='multi_robot_amr_mini').find('multi_robot_amr_mini')

    params_file = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    multi_yaml = os.path.join(pkg_share, 'config', 'multi_twist_mux.yaml')

    with open(params_file, 'r') as file:
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

    twist_mux_node = Node(
        namespace=namespace,
        package='twist_mux',
        executable='twist_mux',
        name="twist_mux",
        parameters=[multi_yaml, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('cmd_vel_out', 'cmd_vel')]
    )

    return [twist_mux_node]


def generate_launch_description():

    return launch.LaunchDescription([

        OpaqueFunction(function=render_xacro)
    ])
