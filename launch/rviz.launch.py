import os
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros


def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='multi_robot_amr_mini').find('multi_robot_amr_mini')    
    rviz_config_file = os.path.join(pkg_share, 'rviz/amr_mini.rviz')

    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return launch.LaunchDescription([
        
        rviz_node,
    ])
