import os
import launch
from launch.actions import IncludeLaunchDescription
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    launch_file_dir = os.path.join(
        get_package_share_directory('multi_robot_amr_mini'), 'launch')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/gazebo.launch.py'])
    )

    merge_laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/merge_laser.launch.py'])
    )

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/twist_mux.launch.py']),
            launch_arguments={"namespace": "amr_mini1"}.items()

    )

    twist_mux_launch2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/twist_mux.launch.py']),
            launch_arguments={"namespace": "amr_mini2"}.items()

    )

    safety_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/safety_system.launch.py'])
    )
 
    return launch.LaunchDescription([
    
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='true',
                                                description='Flag to enable use_sim_time'),

        # gazebo_launch,
        # merge_laser_launch,        
        twist_mux_launch,
        twist_mux_launch2,
        # safety_system_launch

    ])
