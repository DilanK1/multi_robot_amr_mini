import os
from launch import LaunchDescription
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import yaml
from nav2_common.launch import RewrittenYaml
from launch.actions import OpaqueFunction

def render_xacro(context):

    namespace = LaunchConfiguration('namespace').perform(context)
    initial_pose_x = float(LaunchConfiguration('initial_pose_x').perform(context))
    initial_pose_y = float(LaunchConfiguration('initial_pose_y').perform(context))

    yaml_file = os.path.join(get_package_share_directory(
        'multi_robot_amr_mini'), 'config/amcl.yaml')
    multi_yaml = os.path.join(get_package_share_directory(
        'multi_robot_amr_mini'), 'config/multi_amcl.yaml')
    

    lifecycle_nodes = [namespace+'/amcl']
    autostart = True
    
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
            
    initial_pose = {'set_initial_pose': True,
                     'initial_pose': {'x': initial_pose_x, 'y': initial_pose_y, 'z': 0.0}
                     }
    
    param_substitutions = {
        'base_frame_id': namespace+'/base_footprint',
        'odom_frame_id': namespace+'/odom',
    }


    configured_params = RewrittenYaml(
        source_file=multi_yaml,
        param_rewrites=param_substitutions)


    amcl = Node(
        namespace=namespace,
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params, initial_pose]
    )

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'bond_timeout': 0.0},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}
                    ])

    return amcl, start_lifecycle_manager_cmd

def generate_launch_description():

    return LaunchDescription([

        OpaqueFunction(function=render_xacro)
    ])
