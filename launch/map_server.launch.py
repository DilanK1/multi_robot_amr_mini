import os
from launch import LaunchDescription
import launch_ros
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='multi_robot_amr_mini').find('multi_robot_amr_mini')
    map_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    lifecycle_nodes = ['map_server']
    autostart = True

    map_server = Node(
        package = 'nav2_map_server',
        executable = 'map_server',
        name = 'map_server',
        parameters=[{'yaml_filename': map_path},
                    {'use_sim_time' : LaunchConfiguration('use_sim_time')}]
    )

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])
    
    return LaunchDescription([
        map_server,
        start_lifecycle_manager_cmd,
    ]) 