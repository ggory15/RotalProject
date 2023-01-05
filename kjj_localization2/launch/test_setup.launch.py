import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    config = os.path.join(get_package_share_directory('kjj_localization2'),'launch','test_setup.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='kjj_localization2', executable='kjj_localization_node', output='screen',
            name='kjj_localization2_node', parameters = [config])
    ])