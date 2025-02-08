import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name='ros2_automower_description'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot_publish.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    diff_drive = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros2_automower_control"),'launch','diff_drive.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    power_monitor = Node(
        package='ros2_automower_control',
        executable='power_monitor',
        name='power_monitor',
        output='screen'),
    
    blade_controller = Node(
        package='ros2_automower_control',
        executable='blade_controller',
        name='blade_controller',
        output='screen'
    )

    imu = Node(
        package='ros2_automower_localization',
        executable='imu',
        name='imu',
        output='screen'
    )

    sensors = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros2_automower_perception"),'launch','launch_sensors.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )
    # Launch them all!
    return LaunchDescription([
        rsp,
        diff_drive,
        power_monitor,
        blade_controller,
        imu,
        sensors,
    ])