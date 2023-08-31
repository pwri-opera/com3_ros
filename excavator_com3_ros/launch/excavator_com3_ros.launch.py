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
    ns = LaunchConfiguration("ns") 
    can_port = LaunchConfiguration("can_port")
    dbc_path = LaunchConfiguration("dbc_path")

    declare_ns = DeclareLaunchArgument(
        "ns",
        default_value="zx200",
        description=""
    )
    declare_can_port = DeclareLaunchArgument(
        "can_port",
        default_value="vcan0",
        description=""
    )
    declare_dbc_path = DeclareLaunchArgument(
        "dbc_path",
        default_value="/usr/local/share/dbc/excavator_com3.dbc",
        description=""
    )

    # Node
    excavator_com3_relay = Node(
        package="excavator_com3_ros",
        name="excavator_com3_relay",
        namespace=ns,
        executable="excavator_com3_relay",
        parameters=[{"can_port": can_port, "dbc_path": dbc_path}],
        output="screen",
    )

    ld = LaunchDescription()
    # To declare params
    ld.add_action(declare_ns)
    ld.add_action(declare_can_port)
    ld.add_action(declare_dbc_path)
    # To run Node
    ld.add_action(excavator_com3_relay)

    return ld
