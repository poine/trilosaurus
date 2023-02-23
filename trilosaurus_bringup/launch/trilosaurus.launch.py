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

launch_viz = False
launch_teleop = True
#launch_camera = False
#launch_lidar = True

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="trilosaurus_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="trilosaurus_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="trilosaurus_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="trilosaurus.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed then also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="trilosaurus_base_controller",
            description="Robot controller to start.",
        )
    )

    # peripherals
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_camera",
            default_value="False",
            description="Start the camera.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_lidar",
            default_value="False",
            description="Start the lidar.",
        )
    )

    nodes = []
    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    robot_controller = LaunchConfiguration("robot_controller")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " "
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
             "stdout": "screen",
             "stderr": "screen",
        },
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),],
    )
    # Optional Visualisation
    if launch_viz:
        rviz_config_file = PathJoinSubstitution(
            [FindPackageShare(description_package), "rviz", "trilosaurus_view_robot.rviz"]
        )
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        )
        # Delay rviz start after `joint_state_broadcaster`
        delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_node],
            )
        )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "-c", "/controller_manager"],
    )
    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    if launch_teleop:
        joy_config = LaunchConfiguration('joy_config')
        joy_dev = LaunchConfiguration('joy_dev')
        joy_config_filepath = LaunchConfiguration('joy_config_filepath')
        declared_arguments.append(DeclareLaunchArgument('joy_config', default_value='ps3'))
        declared_arguments.append(DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'))
        if 0:
            default_joy_config_path = [TextSubstitution(text=os.path.join(
                    get_package_share_directory('teleop_twist_joy'), 'config', '')),
                joy_config, TextSubstitution(text='.config.yaml')]
        else:
            default_joy_config_path = PathJoinSubstitution([FindPackageShare('trilosaurus_bringup'), "config", "teleop_twist_joy_MOCUTE.config.yaml"])
        declared_arguments.append(DeclareLaunchArgument('joy_config_filepath', default_value=default_joy_config_path))
        
        joy_node = Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.15,
                'autorepeat_rate': 20.0,
            }])
        
        teleop_node = Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name='teleop_twist_joy_node',
            parameters=[joy_config_filepath, {'require_enable_button':True}],
            remappings=[("/cmd_vel", "/trilosaurus_base_controller/cmd_vel_unstamped")]
        )

    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        parameters=[{
            "camera":   0,
            "width":  800,
            "height": 600,
            "role":   "viewfinder",
            'format': 'BGR888',
        }],
        condition=IfCondition(LaunchConfiguration("start_camera")),  
    )
    nodes += [camera_node]
        
    lidar_arguments={'serial_port': '/dev/ttyS0', 'lidar_frame': 'laser_link'}
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ldlidar"), '/launch', '/ldlidar.launch.py']),
        launch_arguments=lidar_arguments.items(),
        condition=IfCondition(LaunchConfiguration("start_lidar")),  
    )
    nodes += [lidar_node]
        
    nodes +=  [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]
    if launch_viz:    nodes += [delay_rviz_after_joint_state_broadcaster_spawner]
    if launch_teleop: nodes += [joy_node, teleop_node]
    #if launch_camera: nodes += [camera_node]
    #if launch_lidar:  nodes += [lidar_inc]
    return LaunchDescription(declared_arguments + nodes)
        

