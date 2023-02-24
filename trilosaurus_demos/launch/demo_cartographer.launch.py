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

#ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.025 -publish_period_sec 1.0
#ros2 run cartographer_ros cartographer_node -configuration_directory ~/work/trilosaurus.ros2/trilosaurus_bringup/cfg/cartographer -configuration_basename turtlebot3_lds_2d.lua

def generate_launch_description():
    
    declared_arguments = []
    nodes =  []

    declared_arguments.append(
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
    
    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        arguments=["-resolution", "0.025", "-publish_period_sec", "1.0"],
        output={
             "stdout": "screen",
             "stderr": "screen",
        },
    )
    nodes += [occupancy_grid_node]

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        arguments=["-configuration_directory", "/home/poine/work/trilosaurus.ros2/trilosaurus_bringup/cfg/cartographer",
                   "-configuration_basename", "turtlebot3_lds_2d.lua"],
        output={
             "stdout": "screen",
             "stderr": "screen",
        },
    )
    nodes += [cartographer_node]


    # navigation
    param_dir = "/home/poine/work/trilosaurus.ros2/trilosaurus_bringup/config/trilosaurus_nav2.yaml"
    map_dir = "/home/poine/work/trilosaurus.ros2/trilosaurus_bringup/map/map.yaml"
    use_sim_time = "false"
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    use_slam = 'True'
    nav_nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch_no_lm.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir, 'slam': use_slam}.items(),
        )
    ]
    nodes += nav_nodes
    
    
    return LaunchDescription(declared_arguments + nodes)

