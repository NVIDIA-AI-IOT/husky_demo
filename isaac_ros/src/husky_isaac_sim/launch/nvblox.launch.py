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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import LoadComposableNodes, Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (SetParameter, SetParametersFromFile, SetRemap)
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    bringup_dir = get_package_share_directory('nvblox_examples_bringup')
    base_config_dir = os.path.join(bringup_dir, 'config', 'nvblox')
    specialization_dir = os.path.join(base_config_dir, 'specializations')

    # Config files
    base_config = os.path.join(base_config_dir, 'nvblox_base.yaml')
    dynamics_config = os.path.join(specialization_dir, 'nvblox_dynamics.yaml')
    simulation_config = os.path.join(
        specialization_dir, 'nvblox_isaac_sim.yaml')

    # Conditionals for setup
    setup_for_dynamics = IfCondition(
        LaunchConfiguration('setup_for_dynamics', default='False'))
    setup_for_isaac_sim = IfCondition(
        LaunchConfiguration('setup_for_isaac_sim', default='False'))
    
    # Option to attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    attach_to_shared_component_container_arg = LaunchConfiguration('attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration('component_container_name', default='nvblox_container')

    # If we do not attach to a shared component container we have to create our own container.
    nvblox_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg)
    )
    
    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
            ComposableNode(
            name='nvblox_node',
            package='nvblox_ros',
            plugin='nvblox::NvbloxNode')])

    group_action = GroupAction([

        # Set parameters with specializations
        SetParametersFromFile(base_config),
        SetParametersFromFile(dynamics_config, condition=setup_for_dynamics),
        SetParametersFromFile(simulation_config),
        SetParameter(name='global_frame', value=LaunchConfiguration('global_frame', default='odom')),

        # Remappings for isaac sim data
        SetRemap(src=['depth/image'], dst=['/front/stereo_camera/rgb/depth']),
        SetRemap(src=['depth/camera_info'], dst=['/front/stereo_camera/rgb/camera_info']),
        SetRemap(src=['color/image'], dst=['/front/stereo_camera/rgb/rgb']),
        SetRemap(src=['color/camera_info'], dst=['/front/stereo_camera/rgb/camera_info']),

        # Include the node container
        load_composable_nodes
    ])

    return LaunchDescription([nvblox_container, group_action])
