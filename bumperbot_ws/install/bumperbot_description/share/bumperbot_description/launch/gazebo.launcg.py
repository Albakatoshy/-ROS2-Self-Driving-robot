import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , SetEnvironmentVariable , IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node 
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    bumbeperbot_description_dir = get_package_share_directory("bumperbot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_description"),
            "urdf",
            "bumperbot.urdf.xacro"
        ),
        description='Path to the bumperbot URDF model file'
    )

    # Substitute after argument is declared
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
    )

    gazebo_resource_path = os.path.join(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(bumbeperbot_description_dir).parent.resolve())
            ]

    )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory("ros_gz_sim"),
            "launch",
            "gz_sim.launch.py"
        )
    ]),
        launch_arguments=[
            (
            "gz_args",[" -v 4", " -r" ," -empty.sdf"]
            )
        ]
    )


    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic" , "/robot_description" , 
                   "-name", "bumperbot"]
    )


    

    return LaunchDescription([
        model_arg,  # Declare first
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        gazebo_resource_path,


    ])
