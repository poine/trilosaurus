# Copyright 2023 poine (poinix@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    param_dir = "/home/poine/work/trilosaurus.ros2/trilosaurus_bringup/config/trilosaurus_nav2.yaml"
    map_dir = "/home/poine/work/trilosaurus.ros2/trilosaurus_bringup/map/map.yaml"
    use_sim_time = "false"
    use_slam = 'True'
    nodes =  []
    declared_arguments = []

    declared_arguments.append(     ## Optionaly starts robot hardware
        DeclareLaunchArgument(
            "start_robot",
            default_value="True",
            description="Start the robot.",
        )
    )
    robot_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("trilosaurus_bringup"), '/launch', '/trilosaurus.launch.py']),
        launch_arguments = {'start_lidar':'True'}.items(),
        condition=IfCondition(LaunchConfiguration("start_robot")),  
    )
    nodes += [robot_nodes]

    declared_arguments.append(     ## SLAM mode
        DeclareLaunchArgument(
            "use_slam",
            default_value="True",
            description="Use slam or localization.",
        )
    )
    use_slam = LaunchConfiguration("use_slam")
    
    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir, 'slam': use_slam}.items(),
        )
    ]
    return LaunchDescription(declared_arguments + nodes + includes)

