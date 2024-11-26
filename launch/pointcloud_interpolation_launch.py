import os
import subprocess
from launch import LaunchDescription
from launch.actions import LogInfo, IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def cleanup_existing_process(context, *args, **kwargs):
    # Check if port 8765 is in use and terminate the process
    try:
        result = subprocess.run(
            ['lsof', '-t', '-i:8765'],
            stdout=subprocess.PIPE,
            text=True,
            check=False
        )
        pid = result.stdout.strip()
        if pid:
            subprocess.run(['kill', pid], check=False)
            print(f"Terminated process using port 8765 (PID: {pid}).")
        else:
            print("No process was using port 8765.")
    except Exception as e:
        print(f"Error during cleanup: {e}")

def generate_launch_description():
    # Declare a launch argument for the log level
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Set the ROS 2 logging level (e.g., debug, info, warn, error, fatal)'
    )

    # Use the user-provided log level or the default value
    log_level = LaunchConfiguration('log_level')

    config = os.path.join(
        get_package_share_directory('dynamic_lidar_interpolation'),
        'config',
        'interpolation_config.yaml'
    )

    foxglove_launch_file = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml'
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_launch_file)
    )

    pointcloud_interpolation_node = Node(
        package='dynamic_lidar_interpolation',
        executable='pointcloud_interpolation_node',
        name='pointcloud_interpolation_node',
        output='screen',
        parameters=[config],
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        log_level_arg,
        OpaqueFunction(function=cleanup_existing_process),
        LogInfo(msg="Launching foxglove_bridge on ws://localhost:8765"),
        foxglove_bridge_launch,
        pointcloud_interpolation_node,
    ])
