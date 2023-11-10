
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def render_xacro(context):
     
    namespace = LaunchConfiguration('namespace').perform(context)
   
    topic_in1 = namespace + "/front_lidar_amr_mini_laser"
    topic_in2 = namespace +"/back_lidar_amr_mini_laser"
    topic_out = namespace +"/scan"

    launch_arguments = {
        'namespace': namespace,
        'topic_in1': topic_in1,
        'topic_in2': topic_in2,
        'topic_out': topic_out,
    }

    laser_relay_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("laser_relay"), '/launch', '/laser_relay.launch.py']),
            launch_arguments = launch_arguments.items()        
    )

    return [laser_relay_launch]

def generate_launch_description():
    
    return LaunchDescription([

        OpaqueFunction(function=render_xacro)

    ])
