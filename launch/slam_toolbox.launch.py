import os
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import launch_ros
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='multi_robot_amr_mini').find('multi_robot_amr_mini')

    slam_launch_file_dir = os.path.join(get_package_share_directory('slam_toolbox'),'launch')
    params_file_dir = os.path.join(pkg_share,'config/mapper_params_online_async.yaml')


    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_launch_file_dir, '/online_async_launch.py']),
        launch_arguments = {'use_sim_time' : 'true', 'params_file' : params_file_dir}.items(),
    )

    return launch.LaunchDescription([

        slam_launch
    ])

