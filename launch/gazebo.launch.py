import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.substitutions import  LaunchConfiguration
from launch.actions import OpaqueFunction
import ast

def render_xacro(context):
    robot_number = (LaunchConfiguration('robot_number').perform(context))
    pose_list =  ast.literal_eval(LaunchConfiguration('pose_list').perform(context))
    liste = list()

    for i in range(int(robot_number)):

        entity_name = "amr_mini" + str(i+1)
        initial_pose_x = (pose_list[i]['initial_pose_x'])
        initial_pose_y = (pose_list[i]['initial_pose_y'])

        print(entity_name)
        
        xacro_urdf_path = os.path.join(get_package_share_directory(
            'multi_robot_amr_mini'), 'urdf_amr', 'amr_mini.xacro')

        robot_desc = xacro.process_file(xacro_urdf_path, mappings={
                                        "namespace": entity_name}).toxml()
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            remappings=[
                ('robot_description', f'{entity_name}/robot_description')],
            parameters=[{'frame_prefix': entity_name+'/', 'use_sim_time': LaunchConfiguration(
                'use_sim_time'), 'robot_description':  robot_desc}]
        )
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            remappings=[
                ('robot_description', f'{entity_name}/robot_description')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', entity_name,
                '-topic', entity_name+"/robot_description",
                '-x', initial_pose_x, '-y', initial_pose_y, '-z', '0.0',
                    '-Y', '0.0', '-P', '0.0', '-R', '0.0',
            ],
            output='screen'
        )

        liste.append(robot_state_publisher_node)
        liste.append(joint_state_publisher_node)
        liste.append(spawn_entity)

    print(len(liste))
    return liste


def generate_launch_description():

    world_file_name = 'my_world.world'
    world = os.path.join(get_package_share_directory(
        'multi_robot_amr_mini'), 'world', world_file_name)

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, 'verbose': "false"}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    return LaunchDescription([


        gzserver,
        gzclient,
        OpaqueFunction(function=render_xacro),

    ])
