from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    use_python_argument = DeclareLaunchArgument(
        "use_python", default_value="True"
    )

    wheel_radius_argument = DeclareLaunchArgument(
        "wheel_radius", default_value="0.033"
    )

    wheel_separation_argument = DeclareLaunchArgument(
        "wheel_separation", default_value="0.17"
    )

    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller", "--controller-manager", "/controller_manager"],
    )

    simple_controller_python = Node(
        package="bumperbot_controller",
        executable="simple_controller.py",
        name="simple_controller",
        parameters=[
            {"wheel_radius": wheel_radius},
            {"wheel_separation": wheel_separation},
        ],
        condition=IfCondition(use_python),
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        simple_controller,
        use_python_argument,
        wheel_radius_argument,
        wheel_separation_argument,
        simple_controller_python,
    ])