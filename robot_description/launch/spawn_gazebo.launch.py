from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import subprocess
import os
import time

def launch_setup(context, *args, **kwargs):
    pkg_path = FindPackageShare('robot_description').find('robot_description')
    urdf_xacro = os.path.join(pkg_path, 'urdf', 'robot_6dof_arm.xacro')
    tmp_urdf = '/tmp/robot_6dof_arm.urdf'

    print("\n>>> Generating URDF from xacro...")
    # Note: We still need xacro even if Gazebo is running separately
    subprocess.run([FindExecutable(name='xacro').perform(context), urdf_xacro, '-o', tmp_urdf])
    os.chmod(tmp_urdf, 0o666)

    # --- GAZEBO LAUNCH REMOVED ---
    # print(">>> Launching Ignition Gazebo...")
    # gz_ros_pkg_path = get_package_share_directory('gz_ros')
    # gazebo = IncludeLaunchDescription(...)
    # -----------------------------

    # Spawn robot entity (needs Gazebo running)

    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ' -r -v 4 empty.sdf'}.items()
    )

    print(">>> Spawning robot entity...")
    spawn_robot = Node(
        package='ros_gz_sim', # Note: Still uses ros_ign_gazebo for spawning
        executable='create',
        name='spawn_robot',
        arguments=['--topic', "robot_description", '--name', 'robot_6dof_arm'],
        output='screen'
    )

    ros_gz_bridge=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState[gz.msgs.JointState',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   ],
        output='screen'
    )

    # Robot State Publisher (TF)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': subprocess.getoutput(f"xacro {urdf_xacro}"),
                     "use_sim_time": True}],
        output='screen'
    )

    controller_manager=Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': subprocess.getoutput(f"xacro {urdf_xacro}")},
                     os.path.join(get_package_share_directory('robot_description'), 'config', 'controllers.yaml')]
    )

    # Spawn Controllers (needs controller_manager running in Gazebo)
    print(">>> Spawning controllers...")
    # Add a small delay before spawning controllers to ensure Gazebo plugin is ready
    spawn_joint_state_broadcaster = TimerAction(period=5.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )])

    spawn_arm_controller = TimerAction(period=7.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )])

    spawn_gripper_controller = TimerAction(period=12.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output='screen',
    )])

    return [
        # gazebo, # REMOVED
        spawn_robot,
        robot_state_pub,
        controller_manager,
        gazebo,
        ros_gz_bridge,
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
        spawn_gripper_controller
    ]

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
