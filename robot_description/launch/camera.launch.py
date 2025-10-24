from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    # --- Generate robot_description content ---
    robot_description_content = ParameterValue(Command(["xacro ", LaunchConfiguration("urdf_path_file")]))
    
    # --- Gazebo Launch ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ' -r -v 4 empty.sdf'}.items()
    )

    # --- Robot State Publisher (TF) ---
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }],
        output='screen'
    )

    # --- Spawn robot entity ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=['--topic', 'robot_description', '--name', 'robot_6dof_arm'],
        output='screen'
    )

    # --- ros_gz Bridges ---
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/overhead_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # --- Controller Manager ---
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {
                'robot_description': robot_description_content,
                'use_sim_time': True
            },
            os.path.join(get_package_share_directory('robot_description'),"robot_description", 'config', 'controllers.yaml')
        ],
        output='screen'
    )

    # --- Spawn Controllers ---
    spawn_joint_state_broadcaster = TimerAction(
        period=5.0, 
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )

    spawn_arm_controller = TimerAction(
        period=7.0, 
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )

    spawn_gripper_controller = TimerAction(
        period=12.0, 
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )

    return [
        gazebo,
        robot_state_pub,
        spawn_robot,
        ros_gz_bridge,
        controller_manager,
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
        spawn_gripper_controller
    ]


def generate_launch_description():
    # --- Launch Arguments ---
    urdf_arg = DeclareLaunchArgument(
        name='urdf_path_file',
        default_value=os.path.join(
            get_package_share_directory('robot_description'),
            'robot_description',  # <--- this is the inner folder
            'urdf',
            'robot_6dof_arm.urdf.xacro'
        ),
        description='Absolute path to the robot URDF xacro file'
    )

    
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value='empty.sdf',
        description='Gazebo world file'
    )

    return LaunchDescription([
        urdf_arg,
        use_sim_time_arg,
        world_arg,
        OpaqueFunction(function=launch_setup)
    ])