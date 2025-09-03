import os
import xacro
import re

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, OpaqueFunction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)


def generate_robot_description(context: LaunchContext, description_package, description_file, controllers_file):
    """Generate robot description using xacro processing."""

    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    controllers_file_str = context.perform_substitution(controllers_file)

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "urdf", "robot", description_file_str
    )

    # Process xacro with required arguments
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "use_gazebo": "true",
            "gazebo_controllers_file": controllers_file_str
        }
    ).toprettyxml(indent="  ")

    return remove_comments(robot_description)


def robot_nodes_spawner(context: LaunchContext, description_package, description_file, controllers_file):
    """Spawn both robot state publisher and control nodes with shared robot description."""

    robot_description = generate_robot_description(
        context, description_package, description_file, controllers_file
    )

    robot_description_param = {"robot_description": robot_description}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}, robot_description_param, {"publish_frequency":15.0}]
    )

    gazebo_model = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity','robot','-topic','/robot_description'], 
        output='screen'
    )

    return [robot_state_pub_node, gazebo_model]


def controller_spawner(context: LaunchContext):
    """Spawn controller based on robot_controller argument."""
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"]
    )

    return [robot_controller_spawner]


def generate_launch_description():
    """Generate launch description for OpenArm bimanual configuration."""

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="robot_description",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="robot.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="robot_gazebo",
            description="Package with the controller's configuration in config folder.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="gazebo_robot_controllers.yaml",
            description="Controllers file(s) to use. Can be a single file or comma-separated list of files.",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value="world.world",
            description="Description package with gazebo world files.",
        ),
    ]

    # Initialize launch configurations
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    world_file = LaunchConfiguration("world_file")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    world_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "worlds", world_file]
    )

    start_gazebo_cmd =  OpaqueFunction(
        function=lambda context: [ExecuteProcess(
            cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen'
        )]
    )   

    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[description_package, description_file, controllers_file]
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = OpaqueFunction(
        function=lambda context: [ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
            output='screen'
        )]
    )

    # Controller spawners
    arm_controller_spawner_func = OpaqueFunction(
        function=lambda context: [ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'arm_controller'],
            output='screen'
        )]
    )

    body_controller_spawner_func = OpaqueFunction(
        function=lambda context: [ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'body_controller'],
            output='screen'
        )]
    )

    gripper_controller_spawner = OpaqueFunction(
        function=lambda context: [Node(
            package="controller_manager",
            executable="spawner",
            arguments=["left_arm_gripper_controller",
                       "right_arm_gripper_controller"]
        )]
    )

    diff_controller_spawner = OpaqueFunction(
        function=lambda context: [ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'diff_controller'],
            output='screen'
        )]
    )

    # Timing and sequencing
    LAUNCH_DELAY_SECONDS = 1.0
    delayed_joint_state_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_body_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[body_controller_spawner_func]
    )

    delayed_arm_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[arm_controller_spawner_func]
    )

    delayed_gripper_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[gripper_controller_spawner]
    )

    delayed_diff_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[diff_controller_spawner]
    )

    return LaunchDescription(
        declared_arguments + [
            start_gazebo_cmd,
            robot_nodes_spawner_func,
            delayed_joint_state_controller,
            delayed_body_controller,
            delayed_arm_controller,
            # delayed_gripper_controller,
            # delayed_diff_controller
        ]
    )
