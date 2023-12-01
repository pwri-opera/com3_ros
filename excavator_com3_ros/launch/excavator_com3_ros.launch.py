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
    dead_zone = LaunchConfiguration("dead_zone")

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

    declare_dead_zone = DeclareLaunchArgument(
        "dead_zone",
        # dead_zone for hydraulic actuators is list value and each element description is below:
        # each element must be float value
        # element 0:boom up
        # element 1:boom down
        # element 2:arm crowd
        # element 3:arm dump
        # element 4:bucket crowd
        # element 5:bucket dump
        # element 6:right swing
        # element 7:left swting
        # element 8:right track forward
        # element 9:right track backward
        # element 10:left track forward
        # element 11:left track backward
        default_value='[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0., 0., 0., 0.]',
        description="Dead zone values for hydraulic actuator"
    )

    # Node
    excavator_com3_relay = Node(
        package="excavator_com3_ros",
        name="excavator_com3_relay",
        namespace=ns,
        executable="excavator_com3_relay",
        parameters=[{"can_port": can_port, "dbc_path": dbc_path, "dead_zone": dead_zone}],
        output="screen",
    )

    ld = LaunchDescription()
    # To declare params
    ld.add_action(declare_ns)
    ld.add_action(declare_can_port)
    ld.add_action(declare_dbc_path)
    ld.add_action(declare_dead_zone)
    # To run Node
    ld.add_action(excavator_com3_relay)

    return ld
