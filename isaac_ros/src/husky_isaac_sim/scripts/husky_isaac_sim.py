# Copyright (C) 2022, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import carb
from omni.isaac.kit import SimulationApp
import sys

CONFIG = {"renderer": "RayTracedLighting", "headless": False}
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

simulation_app = SimulationApp(CONFIG)

from omni.isaac.core import World, SimulationContext
from omni.isaac.core.utils import stage, nucleus
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import is_stage_loading
import omni.graph.core as og
from omni.kit import commands
from omni import usd
from omni.isaac.core_nodes.scripts.utils import set_target_prims

# enable ROS bridge extension
enable_extension("omni.isaac.ros2_bridge-humble")

simulation_app.update()

# Note that this is not the system level rclpy, but one compiled for omniverse
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
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
        self.ros_sub = self.create_subscription(String, "robot_description", self.callback_description, 1)
        self.ros_sub  # prevent unused variable warning
        #self.ros_sub_vel = self.create_subscription(Twist, "cmd_vel", self.callback_test, 1)
        #self.ros_sub_vel  # prevent unused variable warning
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
        status, stage_path = commands.execute(
            "URDFParseAndImportFile",
            urdf_path=PATH_LOCAL_URDF_FOLDER,
            import_config=import_config,
        )
        # Wait a step
        self.isaac_world.wait_step_reload()
        
        # Creating a action graph with ROS component nodes
        # https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_python.html
        # https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_turtlebot.html#build-the-graph
        try:
            (ros_camera_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": f"/{robot_name}/ActionGraph",
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                        ("ROS2SubscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                        ("scale_to_from_stage_units", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                        ("break_3_vector_01", "omni.graph.nodes.BreakVector3"),
                        ("break_3_vector_02", "omni.graph.nodes.BreakVector3"),
                        ("DifferentialController", "omni.isaac.wheeled_robots.DifferentialController"),
                        ("array_index_01", "omni.graph.nodes.ArrayIndex"),
                        ("array_index_02", "omni.graph.nodes.ArrayIndex"),
                        ("ConstantToken_01", "omni.graph.nodes.ConstantToken"),
                        ("ConstantToken_02", "omni.graph.nodes.ConstantToken"),
                        ("ConstantToken_03", "omni.graph.nodes.ConstantToken"),
                        ("ConstantToken_04", "omni.graph.nodes.ConstantToken"),
                        ("MakeArray", "omni.graph.nodes.MakeArray"),
                        ("MakeArray_02", "omni.graph.nodes.MakeArray"),
                        ("IsaacArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ROS2SubscribeTwist.inputs:execIn"),
                        ("ROS2Context.outputs:context", "ROS2SubscribeTwist.inputs:context"),
                        ("ROS2SubscribeTwist.outputs:angularVelocity", "break_3_vector_01.inputs:tuple"),
                        ("ROS2SubscribeTwist.outputs:linearVelocity", "scale_to_from_stage_units.inputs:value"),
                        ("scale_to_from_stage_units.outputs:result", "break_3_vector_02.inputs:tuple"),
                        ("ROS2SubscribeTwist.outputs:execOut", "DifferentialController.inputs:execIn"),
                        ("break_3_vector_01.outputs:z", "DifferentialController.inputs:angularVelocity"),
                        ("break_3_vector_02.outputs:x", "DifferentialController.inputs:linearVelocity"),
                        ("OnPlaybackTick.outputs:tick", "IsaacArticulationController.inputs:execIn"),
                        ("DifferentialController.outputs:velocityCommand", "array_index_01.inputs:array"),
                        ("DifferentialController.outputs:velocityCommand", "array_index_02.inputs:array"),
                        ("array_index_01.outputs:value", "MakeArray_02.inputs:a"),
                        ("array_index_01.outputs:value", "MakeArray_02.inputs:c"),
                        ("array_index_02.outputs:value", "MakeArray_02.inputs:b"),
                        ("array_index_02.outputs:value", "MakeArray_02.inputs:d"),
                        ("MakeArray_02.outputs:array", "IsaacArticulationController.inputs:velocityCommand"),
                        # ("DifferentialController.outputs:velocityCommand", "IsaacArticulationController.inputs:velocityCommand"),
                        ("ConstantToken_01.inputs:value", "MakeArray.inputs:a"),
                        ("ConstantToken_02.inputs:value", "MakeArray.inputs:b"),
                        ("ConstantToken_03.inputs:value", "MakeArray.inputs:c"),
                        ("ConstantToken_04.inputs:value", "MakeArray.inputs:d"),
                        ("MakeArray.outputs:array", "IsaacArticulationController.inputs:jointNames"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        # Assigning a Domain ID of 1 to Context node
                        ("ROS2Context.inputs:domain_id", 0),
                        # Assigning topic name to clock publisher
                        ("ROS2SubscribeTwist.inputs:topicName", "/cmd_vel"),
                        # Assigning Differential controller configuration
                        ("DifferentialController.inputs:maxLinearSpeed", 10000.0),
                        ("DifferentialController.inputs:wheelDistance", 0.512),
                        ("DifferentialController.inputs:wheelRadius", 0.1651),
                        # Assign Articulation controller configuration
                        ("IsaacArticulationController.inputs:usePath", False),
                        # Set size array
                        ("array_index_01.inputs:index", 0),
                        ("array_index_02.inputs:index", 1),
                        ("MakeArray.inputs:arraySize", 4),
                        ("MakeArray_02.inputs:arraySize", 4),
                        # Assigning topic name to clock publisher
                        ("ConstantToken_01.inputs:value", "rear_left_wheel_joint"),
                        ("ConstantToken_02.inputs:value", "rear_right_wheel_joint"),
                        ("ConstantToken_03.inputs:value", "front_left_wheel_joint"),
                        ("ConstantToken_04.inputs:value", "front_right_wheel_joint"),
                    ]
                },
            )
        except Exception as e:
            print(e)
        
        HUSKY_STAGE_PATH=f"/{robot_name}/base_link"
        # Setting the /Franka target prim to Subscribe JointState node
        set_target_prims(primPath=f"/{robot_name}/ActionGraph/IsaacArticulationController", targetPrimPaths=[HUSKY_STAGE_PATH])

        # Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
        # og.Controller.evaluate_sync(ros_camera_graph)

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
    isaac_world = IsaacWorld()#BACKGROUND_USD_PATH)
    # Start simulation
    #isaac_world.start_simulation()
    # Initialize robot loader
    robot_loader = RobotLoader(isaac_world)
    # Run simulation
    isaac_world.run_simulation(robot_loader)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_loader.destroy_node()
    rclpy.shutdown()
# EOF