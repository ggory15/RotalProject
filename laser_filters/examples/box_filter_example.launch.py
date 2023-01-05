from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([

        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            remappings=[
                ("scan", "scan1"),
                ("scan_filtered", "scan1_filtered"),
            ],
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("laser_filters"),
                    "examples", "box_filter_example.yaml",
                ])],
        ),
        
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            remappings=[
                ("scan", "scan2"),
                ("scan_filtered", "scan2_filtered"),
            ],
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("laser_filters"),
                    "examples", "box_filter_example.yaml",
                ])],
        )

        
    ])


