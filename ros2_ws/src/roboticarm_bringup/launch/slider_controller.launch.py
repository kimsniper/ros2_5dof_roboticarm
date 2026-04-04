import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    is_sim = LaunchConfiguration("is_sim")

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("roboticarm_bringup"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={"is_sim": LaunchConfiguration("is_sim")}.items(),
            condition=IfCondition(is_sim)
        )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        remappings=[
            ("/joint_states", "/joint_commands"),
        ]
    )

    slider_control_node = Node(
        package="roboticarm_controller",
        executable="slider_control"
    )

    return LaunchDescription(
        [
            is_sim_arg,
            controller,
            joint_state_publisher_gui_node,
            slider_control_node
        ]
    )
