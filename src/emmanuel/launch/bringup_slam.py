# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'sensors.launch.py']
    )

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'joy_teleop.launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    # ===========================================
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'config', 'slam.yaml']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'rviz', 'linorobot2_slam.rviz']
    )

    ros_distro = EnvironmentVariable('ROS_DISTRO')
    slam_param_name = 'params_file'
    if ros_distro == 'galactic':
        slam_param_name = 'slam_params_file'

    return LaunchDescription([
        DeclareLaunchArgument(
            name='base_serial_port',
            default_value='/dev/ttyACM0',
            description='Linorobot Base Serial Port'
        ),

        DeclareLaunchArgument(
            name='joy',
            default_value='false',
            description='Use Joystick'
        ),

        # Node(
        #     package='micro_ros_agent',
        #     executable='micro_ros_agent',
        #     name='micro_ros_agent',
        #     output='screen',
        #     arguments=['serial', '--dev', LaunchConfiguration("base_serial_port")]
        # ),

        # Writing our custom node
        Node(
            package='emmanuel',
            executable='motor_controller',
            name='motor_controller',
            output='screen'
        ),

        Node(
            package='emmanuel',
            executable='servo_controller',
            name='servo_controller',
            output='screen'
        ),

        Node(
            package='emmanuel',
            executable='speaker_controller',
            name='speaker_controller',
            output='screen'
        ),

        Node(
            package='emmanuel',
            executable='camera_stream',
            name='camera_stream',
            output='screen'
        ),

        Node(
            package='emmanuel',
            executable='server_goal_listener',
            name='server_goal_listener',
            output='screen'
        ),

        # ==========================

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch_path),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
            condition=IfCondition(LaunchConfiguration("joy")),
        ),

        # ==============================================
        DeclareLaunchArgument(
            name='sim',
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='Run rviz'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                slam_param_name: slam_config_path
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        )
    ])
