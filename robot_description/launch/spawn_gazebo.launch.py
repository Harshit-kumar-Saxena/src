from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
import subprocess
import os
import time

def launch_setup(context, *args, **kwargs):
    pkg_path = FindPackageShare('robot_description').find('robot_description')
    urdf_xacro = os.path.join(pkg_path, 'urdf', 'robot_6dof_arm.xacro')
    tmp_urdf = '/tmp/robot_6dof_arm.urdf'

    print("\n>>> Generating URDF from xacro...")
    subprocess.run([FindExecutable(name='xacro').perform(context), urdf_xacro, '-o', tmp_urdf])
    os.chmod(tmp_urdf, 0o666)

    print(">>> Launching Ignition Gazebo...")
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '3', 'empty.sdf'],
        output='screen'
    )

    # Wait until world is loaded and then spawn robot
    print(">>> Waiting for Gazebo to initialize...")
    time.sleep(5)  # safe delay
    print(">>> Spawning robot entity...")

    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_robot',
        arguments=['-file', tmp_urdf, '-entity', 'robot_6dof_arm'],
        output='screen'
    )

    # Robot State Publisher (TF)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': subprocess.getoutput(f"xacro {urdf_xacro}")}],
        output='screen'
    )

    # Optional GUIs
    joint_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return [gazebo, spawn_robot, robot_state_pub, joint_gui, rviz]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])









# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import Command, FindExecutable
# from launch.actions import ExecuteProcess


# def generate_launch_description():
#     pkg_path = FindPackageShare('robot_description')
#     urdf_file = PathJoinSubstitution([pkg_path, 'urdf', 'robot_6dof_arm.xacro'])

#     return LaunchDescription([
#         # 1. Start Ignition Gazebo
        
#         ExecuteProcess(
#             cmd=['ign', 'gazebo', '-r', '-v', '4', 'empty.sdf'],
#             output='screen'
#         ), 

#         # Spawn robot in Ignition Gazebo
#         Node(
#             package='ros_ign_gazebo',
#             executable='create',
#             name='spawn_robot',
#             arguments=['-topic', 'robot_description', '-entity', 'robot_6dof_arm'],
#             output='screen'
#         ),

#         # Robot state publisher
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             parameters=[{'robot_description': Command([FindExecutable(name='xacro'), ' ', urdf_file])}],
#             output='screen'
#         ),

#         # Joint state publisher GUI (optional)
#         Node(
#             package='joint_state_publisher_gui',
#             executable='joint_state_publisher_gui',
#             name='joint_state_publisher_gui',
#             output='screen'
#         ),

#         # RViz2 (optional)
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             output='screen',
#             arguments=['-d', PathJoinSubstitution([pkg_path, 'rviz', 'view_config.rviz'])]
#         )
#     ])
