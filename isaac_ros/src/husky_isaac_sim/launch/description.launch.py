# SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.


import os


from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    husky_description = FindPackageShare("husky_description")
    husky_isaac_sim = FindPackageShare("husky_isaac_sim")

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Get URDF via xacro
    xacro_path = PathJoinSubstitution([husky_isaac_sim, "urdf", "husky.isaac.xacro"])
    
    if os.path.exists("/workspaces/isaac_ros-dev/src/host_path"):
        # Load host path (Only for Docker)
        host_path = open("/workspaces/isaac_ros-dev/src/host_path", 'r').readline().rstrip('\n')
        path_meshes = f"{host_path}/isaac_ros/src/husky/husky_description/meshes"
        path_mesh_accessories = f"{host_path}/isaac_ros/src/husky_isaac_sim/meshes"
    else:
        path_meshes = PathJoinSubstitution([husky_description, "meshes"])
        path_mesh_accessories = PathJoinSubstitution([husky_isaac_sim, "meshes"])

    # Launch Robot State Publisher
    isaac_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="isaac_state_publisher",
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(
                         [
                             'xacro ', xacro_path, ' ',
                             'path_meshes:=', path_meshes, ' ',
                             'path_mesh_accessories:=', path_mesh_accessories, ' ',
                         ])
                     }],
        remappings=[('robot_description', 'isaac_description')]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(
                         [
                             'xacro ', xacro_path, ' ',
                         ])
                     }],
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_cmd)
    ld.add_action(isaac_state_publisher_node)
    ld.add_action(robot_state_publisher_node)

    return ld
# EOF
