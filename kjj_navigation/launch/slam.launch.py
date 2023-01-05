# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ros_distro = EnvironmentVariable('ROS_DISTRO')
    slam_param_name = 'params_file'
    if ros_distro == 'galactic': 
        slam_param_name = 'slam_params_file'

    config_file_path = os.path.join(get_package_share_directory('kjj_bringup'), 'config', 'params.yaml')

    nav2_param_file_path = os.path.join(get_package_share_directory('kjj_navigation'), 'config', 'nav2_params.yaml')

    map_param_file_path = os.path.join(get_package_share_directory('kjj_navigation'), 'config', 'mapper_params_online_async.yaml')

    rviz_config_file_path = os.path.join(get_package_share_directory('kjj_navigation'), 'rviz', 'slam.rviz')

    #sensors_launch_path = PathJoinSubstitution([FindPackageShare('linorobot2_bringup'), 'launch', 'sensors.launch.py'])


    #ekf_config_path = PathJoinSubstitution(
    #    [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    #)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_arg_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time ', 
        default_value='false',
        description='Enable use_sime_time to true'
    )

    declare_arg_rviz = DeclareLaunchArgument(
        name='rviz', 
        default_value='false',
        description='Run rviz'
    )



    launch_kjj_description = ExecuteProcess(cmd=["ros2", "launch", "kjj_description", "kjj_description.launch.py"])

    node_kjj_drive = ExecuteProcess(cmd=["ros2", "run", "kjj_drive", "kjj_drive"])

    launch_sick_lidar = ExecuteProcess(cmd=["ros2", "launch", "sick_safetyscanners2", "sick_safetyscanners2_launch.py"], shell=True)

    launch_filter = ExecuteProcess(cmd=["ros2", "launch", "laser_filters", "box_filter_example.launch.py"], shell=True)

    launch_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch', '/navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_file_path}.items(),
    )

    launch_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch', '/online_async_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': map_param_file_path}.items(),
    )

    node_rviz = Node(
           package='rviz2',
           executable='rviz2',
           name='rviz2',
           arguments=['-d', rviz_config_file_path],
           parameters=[{'use_sim_time': use_sim_time}],
           output='screen')

  

    ld = LaunchDescription()

    ld.add_action(declare_arg_use_sim_time)
    ld.add_action(declare_arg_rviz)

    
    ld.add_action(launch_kjj_description)
    ld.add_action(node_kjj_drive)
    ld.add_action(launch_sick_lidar)
    ld.add_action(launch_filter)
    ld.add_action(launch_nav)
    ld.add_action(launch_slam_toolbox)
    ld.add_action(node_rviz)
    

    return ld