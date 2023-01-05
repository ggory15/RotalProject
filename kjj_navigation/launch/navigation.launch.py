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

    launch_dir_kjj_navigation = os.path.join(get_package_share_directory('kjj_navigation'), 'launch')

    config_file_path = os.path.join(get_package_share_directory('kjj_bringup'), 'config', 'params.yaml')

    nav2_param_file_path = os.path.join(get_package_share_directory('kjj_navigation'), 'config', 'navigation.yaml')

    map_file_path = os.path.join(get_package_share_directory('kjj_navigation'), 'map', 'test.yaml')

    #map_param_file_path = os.path.join(get_package_share_directory('kjj_navigation'), 'config', 'mapper_params_online_async.yaml')

    rviz_config_file_path = os.path.join(get_package_share_directory('kjj_navigation'), 'rviz', 'navigation.rviz')

    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')

    #sensors_launch_path = PathJoinSubstitution([FindPackageShare('linorobot2_bringup'), 'launch', 'sensors.launch.py'])

    autostart = 'True'
    namespace = ''
    slam = 'False'
    use_namespace = 'False'
    use_rviz = 'True'
    use_composition = 'True'
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    #ekf_config_file_path = PathJoinSubstitution(
    #    [FindPackageShare("kjj_navigation"), "config", "ekf.yaml"]
    #)
    robot_localization_file_path = os.path.join(get_package_share_directory('kjj_navigation'), 'config', 'ekf.yaml') 

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    config_map = LaunchConfiguration('map', default=map_file_path)

    declare_arg_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time ', 
        default_value='false',
        description='Enable use_sime_time to true'
    )

    declare_arg_rviz = DeclareLaunchArgument(
        name='rviz', 
        default_value='False',
        description='Run rviz'
    )

    declare_arg_map = DeclareLaunchArgument(
        name='map', 
        default_value=config_map,
        description='Navigation map path'
    )



    launch_kjj_description = ExecuteProcess(cmd=["ros2", "launch", "kjj_description", "kjj_description.launch.py"])

    node_kjj_drive = ExecuteProcess(cmd=["ros2", "run", "kjj_drive", "kjj_drive"])

    launch_sick_lidar = ExecuteProcess(cmd=["ros2", "launch", "sick_safetyscanners2", "sick_safetyscanners2_launch.py"], shell=True)

    launch_filter = ExecuteProcess(cmd=["ros2", "launch", "laser_filters", "box_filter_example.launch.py"], shell=True)

    '''
    launch_nav = IncludeLaunchDescription(
        #PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch', '/navigation_launch.py']),
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch', '/bringup_launch.py']),
        launch_arguments={
            'map': config_map,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_file_path}.items(),
    )

    launch_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch', '/online_async_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': map_param_file_path}.items(),
    )
    '''

    '''
    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir_kjj_navigation, 'slam_launch.py')),
        condition=IfCondition(slam),
        launch_arguments={'namespace': namespace,
                            'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'params_file': nav2_param_file_path}.items(),
    )

    launch_localiztion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir_kjj_navigation, 'localization_launch.py')),
        condition=IfCondition(PythonExpression(['not ', slam])),
        launch_arguments={'namespace': namespace,
                            'map': map_file_path,
                            'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'params_file': nav2_param_file_path,
                            'use_lifecycle_mgr': 'false'}.items()
    )

    # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir_kjj_navigation, 'bringup_launch.py')),
        launch_arguments = {'namespace': namespace,
                            'use_namespace': use_namespace,
                            'slam': slam,
                            'map': map_file_path,
                            'use_sim_time': use_sim_time,
                            'params_file': nav2_param_file_path,
                            'default_bt_xml_filename': behavior_tree_xml_path,
                            'autostart': autostart}.items())
    '''
    '''
    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}])
    '''

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}])

    
    
    '''
    node_rviz = Node(
           package='rviz2',
           executable='rviz2',
           name='rviz2',
           arguments=['-d', rviz_config_file_path],
           parameters=[{'use_sim_time': use_sim_time}],
           output='screen')
    '''

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir_kjj_navigation, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file_path}.items())



    # bringup_launch
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir_kjj_navigation, 'bringup_launch.py')),
        launch_arguments = {'namespace': namespace,
                            'use_namespace': use_namespace,
                            'slam': slam,
                            'map': config_map,
                            'use_sim_time': use_sim_time,
                            'params_file': nav2_param_file_path,
                            'use_composition': use_composition,
                            #'default_bt_xml_filename': behavior_tree_xml_path,
                            'autostart': autostart}.items())

  


    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_arg_use_sim_time)
    ld.add_action(declare_arg_rviz)
    ld.add_action(declare_arg_map)
    

    

    
    ld.add_action(launch_kjj_description)
    ld.add_action(node_kjj_drive)
    ld.add_action(launch_sick_lidar)
    #ld.add_action(launch_filter)

    #ld.add_action(start_robot_localization_cmd)
    #ld.add_action(launch_localiztion)

    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    
    
    
    

    return ld