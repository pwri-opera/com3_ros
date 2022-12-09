import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # Define arguments for launch files
    can_port = LaunchConfiguration("can_port")
    dbc_path = LaunchConfiguration("dbc_path")

    declare_can_port = DeclareLaunchArgument(
        "can_port",
        default_value="vcan0",
        description=""
    )
    declare_dbc_path = DeclareLaunchArgument(
        "dbc_path",
        default_value="~/ros2_ws/src/com3_ros/excavator_com3_ros/include/com3/excavator_com3.dbc",
        description=""
    )

    # Node
    lever_cmd_relay = Node(
        package="excavator_com3_ros",
        name="lever_cmd_relay",
        executable="lever_cmd_relay",
        parameters=[{"can_port": can_port, "dbc_path": dbc_path}],
        output="screen",
    )
    # machine_setting_cmd_relay = Node(
    #     package="excavator_com3_ros",
    #     name="machine_setting_cmd_relay",
    #     executable="machine_setting_cmd_relay",
    #     parameters=[{"can_port": can_port, "dbc_path": dbc_path}],
    #     output="screen",
    # )

    ld = LaunchDescription()
    # To declare params
    ld.add_action(declare_can_port)
    ld.add_action(declare_dbc_path)
    # To run Node
    ld.add_action(lever_cmd_relay)
    # ld.add_action(machine_setting_cmd_relay)

    return ld
