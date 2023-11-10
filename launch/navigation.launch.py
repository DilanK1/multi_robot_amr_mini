import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch.actions import OpaqueFunction


def render_xacro(context):
     
    recursion_level = LaunchConfiguration('recursion_level').perform(context)
    print(recursion_level)


def generate_launch_description():

    launch_file_dir = os.path.join(
        get_package_share_directory('multi_robot_amr_mini'), 'launch')

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/ekf.launch.py']),
        launch_arguments={'namespace': 'amr_mini1'}.items()
    )

    ekf_launch2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/ekf.launch.py']),
        launch_arguments={'namespace': 'amr_mini2'}.items()
    )

    launch_arguments = {
        'namespace': 'amr_mini1',
        'initial_pose_x': '0.0',
        'initial_pose_y': '0.0'
    }

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/amcl.launch.py']),
        launch_arguments=launch_arguments.items()
    )
    
    launch_arguments2 = {
        'namespace': 'amr_mini2',
        'initial_pose_x': '0.0',
        'initial_pose_y': '6.0'
    }

    amcl_launch2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/amcl.launch.py']),
        launch_arguments=launch_arguments2.items()
    )

    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/map_server.launch.py'])
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/nav2.launch.py']),  
            launch_arguments={'namespace': 'amr_mini1'}.items()
    )

    nav2_launch2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/nav2.launch.py']),
            launch_arguments={'namespace': 'amr_mini2'}.items()
    )

    return LaunchDescription([

        # OpaqueFunction(function=render_xacro),

        DeclareLaunchArgument(name='use_sim_time', default_value='true',
                              description='Flag to enable use_sim_time'),
        

        ekf_launch,
        ekf_launch2,
        # amcl_launch,
        # amcl_launch2,
        # nav2_launch,
        # nav2_launch2
        # map_server_launch

    ])
