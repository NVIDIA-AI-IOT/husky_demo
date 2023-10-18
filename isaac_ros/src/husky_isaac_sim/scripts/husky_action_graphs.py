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

import omni
import omni.graph.core as og
from omni.isaac.core.utils import stage
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from pxr import Gf, UsdGeom
from omni.isaac.core.utils.prims import set_targets

def create_camera(robot_name, number_camera, camera_frame, camera_stage_path, camera_name, stereo_offset=[0.0, 0.0]):
    
    ros_camera_graph_path = f"/{robot_name}/ROS_CameraGraph_{camera_name}"
    viewport_name = f"Viewport{number_camera}"
    if camera_name == "depth":
        type_camera = "depth"
    else:
        type_camera = "rgb"
    
    # Creating an on-demand push graph with cameraHelper nodes to generate ROS image publishers
    keys = og.Controller.Keys
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": ros_camera_graph_path,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                ("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                ("setViewportResolution", "omni.isaac.core_nodes.IsaacSetViewportResolution"),
                ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                ("cameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                ("createViewport.outputs:execOut", "setViewportResolution.inputs:execIn"),
                ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                ("createViewport.outputs:viewport", "setViewportResolution.inputs:viewport"),
                ("setViewportResolution.outputs:execOut", "setCamera.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                ("setCamera.outputs:execOut", "cameraHelper.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelper.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("createViewport.inputs:name", viewport_name),
                ("createViewport.inputs:viewportId", number_camera),
                ("setViewportResolution.inputs:width", 640),
                ("setViewportResolution.inputs:height", 480),
                ("cameraHelper.inputs:frameId", f"{camera_frame}"),
                ("cameraHelper.inputs:topicName", f"/front/stereo_camera/{camera_name}/{type_camera}"),
                ("cameraHelper.inputs:type", f"{type_camera}"),
                ("cameraHelperInfo.inputs:frameId", f"{camera_frame}"),
                ("cameraHelperInfo.inputs:topicName", f"/front/stereo_camera/{camera_name}/camera_info"),
                ("cameraHelperInfo.inputs:type", "camera_info"),
                ("cameraHelperInfo.inputs:stereoOffset", stereo_offset),
            ],
        },
    )

    set_targets(
        prim=stage.get_current_stage().GetPrimAtPath(ros_camera_graph_path + "/setCamera"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[camera_stage_path],
    )


def build_camera_graph(robot_name,type_output):
    
    camera_color_stage_path = f"/{robot_name}/camera_color_optical_frame/camera_color"
    # Creating a Camera prim
    camera_color_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(camera_color_stage_path, "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_color_prim)
    #xform_api.SetTranslate(Gf.Vec3d(0, 0, 0))
    xform_api.SetRotate((0, -180, -180), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    #camera_color_prim.GetHorizontalApertureAttr().Set(21)
    #camera_color_prim.GetVerticalApertureAttr().Set(16)
    #camera_color_prim.GetProjectionAttr().Set("perspective")
    #camera_color_prim.GetFocalLengthAttr().Set(24)
    #camera_color_prim.GetFocusDistanceAttr().Set(400)

    camera_depth_stage_path = f"/{robot_name}/camera_depth_optical_frame/camera_depth"
    # Creating a Camera prim
    camera_depth_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(camera_depth_stage_path, "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_depth_prim)
    xform_api.SetTranslate(Gf.Vec3d(0.1, 0, 0))
    #xform_api.SetRotate((0, -180, -180), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    xform_api.SetRotate((0, 0, -180), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    #xform_api.SetRotate((90, -90, 0), UsdGeom.XformCommonAPI.RotationOrderZYX)
    camera_depth_prim.GetHorizontalApertureAttr().Set(2.0955)
    camera_depth_prim.GetVerticalApertureAttr().Set(1.1769)
    camera_depth_prim.GetClippingRangeAttr().Set((0.01, 10000))
    camera_depth_prim.GetProjectionAttr().Set("perspective")
    camera_depth_prim.GetVisibilityAttr().Set("invisible")
    camera_depth_prim.GetFocalLengthAttr().Set(2.4)
    camera_depth_prim.GetFocusDistanceAttr().Set(4)

    camera_infra1_stage_path = f"/{robot_name}/camera_infra1_optical_frame/camera_infra1"
    # Creating a Camera prim
    camera_infra1_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(camera_infra1_stage_path, "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_infra1_prim)
    xform_api.SetTranslate(Gf.Vec3d(0.1, 0, 0))
    #xform_api.SetRotate((0, -180, -180), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    xform_api.SetRotate((0, 0, -180), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    #xform_api.SetRotate((90, -90, 0), UsdGeom.XformCommonAPI.RotationOrderZYX)
    camera_infra1_prim.GetHorizontalApertureAttr().Set(2.0955)
    camera_infra1_prim.GetVerticalApertureAttr().Set(1.1769)
    camera_infra1_prim.GetClippingRangeAttr().Set((0.01, 10000))
    camera_infra1_prim.GetProjectionAttr().Set("perspective")
    camera_infra1_prim.GetVisibilityAttr().Set("invisible")
    camera_infra1_prim.GetFocalLengthAttr().Set(2.4)
    camera_infra1_prim.GetFocusDistanceAttr().Set(4)
    
    camera_infra2_stage_path = f"/{robot_name}/camera_infra2_optical_frame/camera_infra2"
    # Creating a Camera prim
    camera_infra2_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(camera_infra2_stage_path, "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_infra2_prim)
    xform_api.SetTranslate(Gf.Vec3d(0.1, 0, 0))
    #xform_api.SetRotate((0, -180, -180), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    xform_api.SetRotate((0, 0, -180), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    #xform_api.SetRotate((90, -90, 0), UsdGeom.XformCommonAPI.RotationOrderZYX)
    camera_infra2_prim.GetHorizontalApertureAttr().Set(2.0955)
    camera_infra2_prim.GetVerticalApertureAttr().Set(1.1769)
    camera_infra2_prim.GetClippingRangeAttr().Set((0.01, 10000))
    camera_infra2_prim.GetProjectionAttr().Set("perspective")
    camera_infra2_prim.GetVisibilityAttr().Set("invisible")
    camera_infra2_prim.GetFocalLengthAttr().Set(2.4)
    camera_infra2_prim.GetFocusDistanceAttr().Set(4)
    # Create rgb camera
    create_camera(robot_name, 1, "camera_infra1_optical_frame", camera_infra1_stage_path, "left")
    create_camera(robot_name, 2, "camera_infra2_optical_frame", camera_infra2_stage_path, "right", stereo_offset=[-50.0, 0])
    create_camera(robot_name, 3, "camera_depth_optical_frame", camera_depth_stage_path, "depth")

def build_differential_controller_graph(robot_name):   
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
                    ("array_index_01.outputs:value", "MakeArray_02.inputs:c"),
                    ("array_index_01.outputs:value", "MakeArray_02.inputs:a"),
                    ("array_index_02.outputs:value", "MakeArray_02.inputs:d"),
                    ("array_index_02.outputs:value", "MakeArray_02.inputs:b"),
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
                    #("DifferentialController.inputs:maxLinearSpeed", 10000.0),
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
                    ("ConstantToken_01.inputs:value", "rear_right_wheel_joint"),
                    ("ConstantToken_02.inputs:value", "rear_left_wheel_joint"),
                    ("ConstantToken_03.inputs:value", "front_right_wheel_joint"),
                    ("ConstantToken_04.inputs:value", "front_left_wheel_joint"),
                ]
            },
        )
    except Exception as e:
        print(e)
    
    HUSKY_STAGE_PATH=f"/{robot_name}/base_link"
    # Setting the /Franka target prim to Subscribe JointState node
    set_target_prims(primPath=f"/{robot_name}/ActionGraph/IsaacArticulationController", targetPrimPaths=[HUSKY_STAGE_PATH])
# EOF