import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription, Node, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = launch_ros.substitutions/FindPackageShare(package="kinect_ros2").find("kinect_ros2")
    default_rviz_config_path = os.path.join(pkg_share, "rvis/pointcloud.rviz")
    package_name='ros2_automower_perception'


    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            name="rvizconfig"
            default_value=default_rviz_config_path
            description="Absolute path to rviz config file"
        ),
        Node(
            package="kinect_ros2",
            exectuable="kinect_ros2_node",
            name="kinect_ros2",
            namespace="kinect",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rvizconfig")],
        ),
        Node(
            package="image_tools",
            executable="showimage",
            name="rgb_showimage",
            parameters=[{"window_name": "RGB"}],
            remappings=[("image", "kinect/image_raw")],
        ),
        ## SONAR
        # Node(
        #     package='ros2_automower_perception',
        #     executable='sonar',
        #     name='left_sonar',
        #     parameters=[{"echo_pin": 22},
        #                 {"trig_pin": 25},
        #                 {"frame": "left_sonar_link"}]
        # ),
        # Node(
        #     package='ros2_automower_perception',
        #     executable='sonar',
        #     name='left_sonar',
        #     parameters=[{"echo_pin": 22},
        #                 {"trig_pin": 25},
        #                 {"frame": "right_sonar_link"}]
        # ),
        # Node(
        #     package='ros2_automower_perception',
        #     executable='sonar',
        #     name='left_sonar',
        #     parameters=[{"echo_pin": 22},
        #                 {"trig_pin": 25},
        #                 {"frame": "front_left_sonar_link"}]
        # ),
        # Node(
        #     package='ros2_automower_perception',
        #     executable='sonar',
        #     name='left_sonar',
        #     parameters=[{"echo_pin": 22},
        #                 {"trig_pin": 25},
        #                 {"frame": "front_right_sonar_link"}]
        # ),
    ])