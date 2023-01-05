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


MY_KJJ_ENVIRONMENT = 'map2'

def generate_launch_description():
    use_multi_robots = LaunchConfiguration('use_multi_robots', default='False')
    use_amcl = LaunchConfiguration('use_amcl', default='False')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='False')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('kjj_real'),
            'maps',
            MY_KJJ_ENVIRONMENT+'.yaml'))

    param_file_name = 'kjj_navigation.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('kjj_real'),
            'configs/',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('kjj_nav2_bringup'), 'launch')
    merge_launch_file_dir = os.path.join(get_package_share_directory('laser_scan_integrator'), 'launch')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_kjj.launch.py']), 
            condition=IfCondition(PythonExpression(['not ', use_amcl])),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': param_dir,
                'namespace': namespace}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_amcl.launch.py']),
            condition=IfCondition(use_amcl),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': param_dir,
                'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_kjj.launch.py']),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'params_file': param_dir}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([merge_launch_file_dir, '/integrate_2_scan.launch.py']),
            ),
        ])
