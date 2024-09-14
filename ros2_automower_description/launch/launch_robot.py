import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    # Launch them all!
    return LaunchDescription([
        rsp,
        diff_drive
    ])