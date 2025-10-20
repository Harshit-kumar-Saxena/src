from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_path = FindPackageShare("robot_description")
    xacro_file = PathJoinSubstitution([pkg_path, "urdf", "robot_6dof_arm.xacro"])

    robot_description = {"robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file])}

    return LaunchDescription([
        # Publishes TFs for each joint
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen"
        ),

        # GUI sliders to move joints
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen"
        ),

        # Optional RViz visualization
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(PathJoinSubstitution([pkg_path, "rviz", "view_config.rviz"]))]
        )
    ])
