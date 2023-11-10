import os
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription


def generate_launch_description():

    robot_number = '3'
    ld = LaunchDescription()

    pose_list = [{'initial_pose_x': '0.0', 'initial_pose_y': '0.0'}, {
        'initial_pose_x': '0.0', 'initial_pose_y': '6.0'},{'initial_pose_x': '0.0', 'initial_pose_y': '-6.0'}]

    launch_file_dir = os.path.join(
        get_package_share_directory('multi_robot_amr_mini'), 'launch')    
   
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/slam_toolbox.launch.py'])
    )
    
    launch_arguments = {
            'robot_number': robot_number,
            'pose_list': str(pose_list)
        }
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/gazebo.launch.py']),
        launch_arguments=launch_arguments.items()
    )

    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/map_server.launch.py'])
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/rviz.launch.py'])
    )

    navigation_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/navigation_stack.launch.py']),
        launch_arguments=launch_arguments.items()
    )

    sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='true',
                                             description='Flag to enable use_sim_time')

    ld.add_action(sim_time)
    ld.add_action(gazebo_launch)
    ld.add_action(map_server_launch)
    ld.add_action(rviz_launch)
    ld.add_action(navigation_stack_launch)
   
    return ld
   