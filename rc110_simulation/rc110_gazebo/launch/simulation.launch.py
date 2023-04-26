#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument

def generate_launch_description():
    joy_type = LaunchConfiguration('joy_type')
    use_gui = LaunchConfiguration('use_gui')
    root = LaunchConfiguration('root')
    world = LaunchConfiguration('world')
    n = LaunchConfiguration('n')

    joy_type_launch_arg = DeclareLaunchArgument("joy_type", default_value="")
    use_gui_launch_arg = DeclareLaunchArgument("use_gui", default_value=True)
    root_launch_arg = DeclareLaunchArgument("root", default_value="")
    world_launch_arg = DeclareLaunchArgument("world", default_value="")
    n_launch_arg = DeclareLaunchArgument("n", default_value="1")

    world_launch = launch_ros.actions.IncludeLaunchDescription(launch=get_package_share_directory("rc110") + "/launch/world.launch")
    model_launch = launch_ros.actions.IncludeLaunchDescription(launch=get_package_share_directory("rc110") +  "/launch/model.launch")
    return LaunchDescription([
        world_launch,
        model_launch
    ])
