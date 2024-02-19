#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    load_gripper_parameter_name = 'load_gripper'
    use_rviz_parameter_name = 'use_rviz'
    world_parameter_name = 'world'

    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    world = LaunchConfiguration(world_parameter_name)

    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                     'panda_arm.urdf.xacro')
    parameters_file = os.path.join(get_package_share_directory('franka_bringup'), 'config',
                                     'controllers.yaml')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper,
         ' robot_ip:=127.0.0.1', ' use_fake_hardware:=false', ' fake_sensor_commands:=false',
         ' gazebo:=true', ' parameters_file:=', parameters_file])

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')

    franka_controllers = PathJoinSubstitution(
        [
            FindPackageShare('franka_bringup'),
            'config',
            'controllers.yaml',
        ]
    )

    gz_bridge_config_path = PathJoinSubstitution(
        [
            FindPackageShare('franka_gazebo'),
            'config',
            'gz_bridge.yaml',
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller'],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        get_package_share_directory("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ),
            launch_arguments={"gz_args": "-r empty.sdf -v 3"}.items(),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                "panda",
                "-allow_renaming",
                "true",
                "-topic",
                "robot_description",
                # "-x",
                # pose_x,
                # "-y",
                # pose_y,
                # "-z",
                # pose_z,
                # "-R",
                # rot_roll,
                # "-P",
                # rot_pitch,
                # "-Y",
                # rot_yaw,
            ],
            output="screen",
        ),

        SetParameter(
            name="use_sim_time",
            value="true"
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            parameters=[{"config_file": gz_bridge_config_path}],
            output="screen",
        ),

        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
             )

    ])
