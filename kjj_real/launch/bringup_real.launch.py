# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path

MY_KJJ_ROBOT = 'mpo_700'
MY_KJJ_ENVIRONMENT = 'neo_workshop'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    launch_kjj_description = ExecuteProcess(cmd=["ros2", "launch", "kjj_description", "kjj_description.launch.py"])
    node_kjj_drive = ExecuteProcess(cmd=["ros2", "run", "kjj_drive", "kjj_drive"])
    launch_sick_lidar = ExecuteProcess(cmd=["ros2", "launch", "sick_safetyscanners2", "sick_safetyscanners2_launch.py"], shell=True)
    launch_filter = ExecuteProcess(cmd=["ros2", "launch", "laser_filters", "box_filter_example.launch.py"], shell=True)


    teleop =  Node(package='teleop_twist_keyboard',executable="teleop_twist_keyboard",
    output='screen',
    prefix = 'xterm -e',
    name='teleop')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    
    ld.add_action(launch_kjj_description)
    ld.add_action(node_kjj_drive)
    ld.add_action(launch_sick_lidar)
    ld.add_action(launch_filter)
    ld.add_action(teleop)

    return ld