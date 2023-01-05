# Neobotix GmbH
# Author: Pradheep Padmanabhan

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition


MY_KJJ_ROBOT = 'mpo_700'
MY_KJJ_ENVIRONMENT = 'neo_workshop_speed'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('kjj_simulation'),
            'maps',
            MY_KJJ_ENVIRONMENT+'.yaml'))
    
    speed_param_file_name = 'speed_params.yaml'
    speed_param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('nav2_costmap_filters_demo'),
            'params/',
            speed_param_file_name))

    speed_launch_file_dir = os.path.join(get_package_share_directory('nav2_costmap_filters_demo'), 'launch')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([speed_launch_file_dir, '/costmap_filter_info.launch.py']), 
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': speed_param_dir,
                'mask': map_dir}.items(),
        ),       
        ])
