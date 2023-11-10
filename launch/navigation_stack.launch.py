import os
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import OpaqueFunction
import ast


def render_xacro(context):

    robot_number = (LaunchConfiguration('robot_number').perform(context))
    pose_list =  ast.literal_eval(LaunchConfiguration('pose_list').perform(context))

    liste = list()
    
    launch_file_dir = os.path.join(
        get_package_share_directory('multi_robot_amr_mini'), 'launch')

    for i in range(int(robot_number)):

        namespace = 'amr_mini' + str(i+1)

        launch_arguments_amcl = {
            'namespace': namespace,
            'initial_pose_x': pose_list[i]['initial_pose_x'],
            'initial_pose_y': pose_list[i]['initial_pose_y']
        }

        merge_laser_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_dir, '/merge_laser.launch.py']),
            launch_arguments={"namespace": namespace}.items()
        )

        ekf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/ekf.launch.py']),
            launch_arguments={'namespace': namespace}.items()
        )

        twist_mux_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_dir, '/twist_mux.launch.py']),
            launch_arguments={"namespace": namespace}.items()
        )


        amcl_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_dir, '/amcl.launch.py']),
            launch_arguments=launch_arguments_amcl.items(),
        )

        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_dir, '/nav2.launch.py']),
            launch_arguments={'namespace': namespace}.items()
        )

        liste.append(merge_laser_launch)
        liste.append(ekf_launch)
        liste.append(twist_mux_launch)
        liste.append(amcl_launch)
        liste.append(nav2_launch)

        
    return liste


def generate_launch_description():

    return LaunchDescription([

        OpaqueFunction(function=render_xacro),

    ])
