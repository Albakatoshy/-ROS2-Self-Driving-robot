from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Declare launch arguments first
    use_python_argument = DeclareLaunchArgument(
        "use_python", default_value="True"
    )

    wheel_radius_argument = DeclareLaunchArgument(
        "wheel_radius", default_value="0.033"
    )

    wheel_separation_argument = DeclareLaunchArgument(
        "wheel_separation", default_value="0.17"
    )

    use_simple_controller_argument = DeclareLaunchArgument(
        "use_simple_controller", default_value="True"
    )

    # Get launch configurations
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    # Define nodes and actions
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bumperbot_controller", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_simple_controller), 
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["simple_velocity_controller", "--controller-manager", "/controller_manager"],
            ),
            # Python version of simple_controller
            Node(
                package="bumperbot_controller",
                executable="simple_controller.py",
                name="simple_controller",
                parameters=[{"wheel_radius": wheel_radius,
                             "wheel_separation": wheel_separation}],
                condition=IfCondition(use_python),
            ),
        ]
    )

    return LaunchDescription([
        # MUST put arguments first before any nodes that use LaunchConfiguration
        use_python_argument,
        wheel_radius_argument,
        wheel_separation_argument,
        use_simple_controller_argument,
        # Now nodes and actions can safely use LaunchConfiguration
        joint_state_broadcaster_spawner,
        simple_controller,
        wheel_controller_spawner,
    ])