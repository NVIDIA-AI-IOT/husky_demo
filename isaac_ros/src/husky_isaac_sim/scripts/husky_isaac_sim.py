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

import carb
from omni.isaac.kit import SimulationApp
import sys

CONFIG = {"renderer": "RayTracedLighting", "headless": False}
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
# BACKGROUND_USD_PATH = ""

simulation_app = SimulationApp(CONFIG)

from omni.isaac.core import World, SimulationContext
from omni.isaac.core.utils import stage, nucleus
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import is_stage_loading
from omni.kit import commands
from omni.usd import get_stage_next_free_path

from husky_action_graphs import build_differential_controller_graph, build_camera_graph, build_clock_graph

# enable ROS bridge extension
enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

# Note that this is not the system level rclpy, but one compiled for omniverse
import rclpy
from std_msgs.msg import String
from rclpy.node import Node


PATH_LOCAL_URDF_FOLDER="/tmp/robot.urdf"

class IsaacWorld():
    
    def __init__(self, stage_path=""):
        self.commands=[]
        # Setting up scene
        if stage_path:
            self.simulation_context = SimulationContext(stage_units_in_meters=1.0)
            # Locate assets root folder to load sample
            assets_root_path = nucleus.get_assets_root_path()
            if assets_root_path is None:
                carb.log_error("Could not find Isaac Sim assets folder")
                simulation_app.close()
                sys.exit()
            # Loading the simple_room environment
            stage.add_reference_to_stage(assets_root_path + stage_path, BACKGROUND_STAGE_PATH)
        else:
            self.simulation_context = World(stage_units_in_meters=1.0)
            self.simulation_context.scene.add_default_ground_plane()
            # need to initialize physics getting any articulation..etc
            self.simulation_context.initialize_physics()
        # Build clock graph
        build_clock_graph()
        # Wait two frames so that stage starts loading
        simulation_app.update()
        simulation_app.update()
    
    def add_tick(self, function):
        self.commands += [function]
    
    def wait_step_reload(self):
        self.simulation_context.step(render=True)
        print("Loading stage...")
        while is_stage_loading():
            simulation_app.update()
        print("Loading Complete")

    def start_simulation(self):
        self.simulation_context.play()

    def run_simulation(self, node):
        while simulation_app.is_running():
            self.simulation_context.step(render=True)
            # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
            for command in self.commands:
                command()
            rclpy.spin_once(node, timeout_sec=0.0)
            if self.simulation_context.is_playing():
                if self.simulation_context.current_time_step_index == 0:
                    self.simulation_context.reset()
        # Cleanup
        self.simulation_context.stop()
        simulation_app.close()

class RobotLoader(Node):
    
    def __init__(self, isaac_world, namespace=""):
        super().__init__("robot_loader", namespace=namespace)
        # Load isaac_world
        self.namespace = namespace
        self.isaac_world = isaac_world
        # setup the ROS2 subscriber here
        self.ros_sub = self.create_subscription(String, "isaac_description", self.callback_description, 1)
        self.ros_sub  # prevent unused variable warning
        # Node started
        self.get_logger().info("Robot loader start")

    def load_robot(self, robot_name):
        # Setting up import configuration:
        status, import_config = commands.execute(
            "URDFCreateImportConfig")
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = False
        import_config.fix_base = False
        import_config.distance_scale = 1

        # Import URDF, stage_path contains the path to the usd prim in the stage.
        # extension_path = get_extension_path_from_name("omni.isaac.urdf")
        dest_path = "/tmp/husky.usd"
        
        status, stage_path = commands.execute(
            "URDFParseAndImportFile",
            urdf_path=PATH_LOCAL_URDF_FOLDER,
            import_config=import_config,
            #dest_path=dest_path
        )
        # TODO change start position        
        #stage_path = get_stage_next_free_path(
        #    simulation_context.scene.stage,
        #    "/World" + stage_path,
        #    False
        #)
        #robot_prim = simulation_context.scene.stage.OverridePrim(stage_path)
        #robot_prim.GetReferences().AddReference(dest_path)
        
        # Wait a step
        self.isaac_world.wait_step_reload()
        # Build differential controller graph
        build_differential_controller_graph(robot_name)
        # Build camera graph
        build_camera_graph(robot_name)
        simulation_app.update()

    def callback_description(self, msg):
        # callback function to set the cube position to a new one upon receiving a (empty) ROS2 message
        print(f"Load robot")
        robot_urdf = msg.data
        #print(robot_urdf)
        robot_name = "husky"
        text_file = open(PATH_LOCAL_URDF_FOLDER, "w")
        n = text_file.write(robot_urdf)
        text_file.close()
        # Load robot
        self.load_robot(robot_name)


if __name__ == "__main__":
    rclpy.init()
    # Isaac SIM world
    isaac_world = IsaacWorld(BACKGROUND_USD_PATH)
    # Start simulation
    isaac_world.start_simulation()
    # Initialize robot loader
    robot_loader = RobotLoader(isaac_world)
    # Run simulation
    isaac_world.run_simulation(robot_loader)
    # Destroy the node explicitly
    robot_loader.destroy_node()
    rclpy.shutdown()
# EOF