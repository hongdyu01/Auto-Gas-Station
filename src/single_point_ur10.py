# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # start the simulation app, with GUI open

import sys

import carb
import math
import numpy as np
from pxr import Usd, UsdLux, Sdf
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.examples.universal_robots.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.manipulators.grippers import SurfaceGripper


################################################################################################################################
# Asset Root Path
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)

asset_path = "/home/rokey/auto_oil_project/assets/default_world.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World")

################################################################################################################################
# Light
stage = my_world.stage

# Remove all existing lights (including referenced ones)
for prim in stage.Traverse():
    if prim.GetTypeName() in ["DistantLight", "SphereLight", "RectLight", "CylinderLight", "DiskLight"]:
        # Create an override first (important for referenced USD)
        override_prim = stage.OverridePrim(prim.GetPath())
        stage.RemovePrim(prim.GetPath())
#
# Add a default light
light_prim = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/defaultLight"))
light_prim.CreateIntensityAttr(1000)
light_prim.CreateAngleAttr(0.53)

################################################################################################################################
left_gasoline = XFormPrim("/World/left_gasoline_gun")
left_diesel = XFormPrim("/World/left_diesel_gun")
left_lpg = XFormPrim("/World/left_lpg_gun")
right_gasoline = XFormPrim("/World/right_gasoline_gun")
right_diesel = XFormPrim("/World/right_diesel_gun")
right_lpg = XFormPrim("/World/right_lpg_gun")
################################################################################################################################
asset_path = "/home/rokey/auto_oil_project/assets/car.usdc"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Car")
# Car prim 생성 후
car = XFormPrim("/World/Car")
car.set_world_poses(positions=np.array([[0.0, 0.0, 0.0]]) / get_stage_units(), orientations=np.array([[1/math.sqrt(2),0,0,1/math.sqrt(2)]]))
oil_door = XFormPrim("/World/Car/World/oildoor")
################################################################################################################################
# Add Franka
asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
stage.DefinePrim("/World/left_base", "Xform")
stage.DefinePrim("/World/right_base", "Xform")
left_base = XFormPrim("/World/left_base")
right_base = XFormPrim("/World/right_base")
left_base.set_world_poses(np.array([[4, 0, 0]]))
right_base.set_world_poses(np.array([[-2.5, 0, 0]]))
left_base.set_local_scales(np.array([[2.5, 2.5, 2.5]]))
right_base.set_local_scales(np.array([[2.0, 2.0, 2.0]]))

left_ur10 = add_reference_to_stage(usd_path=asset_path, prim_path="/World/left_base/left_ur10")
right_ur10 = add_reference_to_stage(usd_path=asset_path, prim_path="/World/right_base/right_ur10")

left_ur10.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")
right_ur10.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")
left_gripper = SurfaceGripper(
    end_effector_prim_path="/World/left_base/left_ur10/ee_link", surface_gripper_path="/World/left_base/left_ur10/ee_link/SurfaceGripper"
)
right_gripper = SurfaceGripper(
    end_effector_prim_path="/World/right_base/right_ur10/ee_link", surface_gripper_path="/World/right_base/right_ur10/ee_link/SurfaceGripper"
)
my_left_ur10 = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/left_base/left_ur10",
        name="left_ur10",
        end_effector_prim_path="/World/left_base/left_ur10/ee_link",
        gripper=left_gripper,
    )
)
my_right_ur10 = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/right_base/right_ur10",
        name="right_ur10",
        end_effector_prim_path="/World/right_base/right_ur10/ee_link",
        gripper=right_gripper,
    )
)

# left_arm = Articulation(prim_paths_expr="/World/left_ur10", name="left_arm")
# right_arm = Articulation(prim_paths_expr="/World/right_ur10", name="right_arm")

# left_arm.set_world_poses(positions=np.array([[2.5, 0.0, 0.0]]) / get_stage_units())
# right_arm.set_world_poses(positions=np.array([[-2.5, 0.0, 0.0]]) / get_stage_units())
# left_arm.set_local_scales(np.array([[2.0, 2.0, 2.0]]))
# right_arm.set_local_scales(np.array([[2.0, 2.0, 2.0]]))

"""
구멍 위치: (1.4, -1, 1)
주유건 위치: (3.5, -0.7, 0.15)
                0 / 0.7
주유건 크기: (0.5, 0.07, 0.07)
"""

# initialize the world
my_left_ur10.gripper.set_default_state(opened=True)
my_right_ur10.gripper.set_default_state(opened=True)

my_world.reset()


my_left_controller = PickPlaceController(
    name="left_controller", gripper=my_left_ur10.gripper, robot_articulation=my_left_ur10
)
left_articulation_controller = my_left_ur10.get_articulation_controller()

my_right_controller = PickPlaceController(
    name="right_controller", gripper=my_right_ur10.gripper, robot_articulation=my_right_ur10
)
right_articulation_controller = my_right_ur10.get_articulation_controller()

reset_needed = False
task_completed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
        task_completed = False
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_left_controller.reset()
            reset_needed = False
            task_completed = False
        observations = my_world.get_observations()
        actions = my_left_controller.forward(
            picking_position=left_diesel.get_world_poses()[0][0],
            placing_position=oil_door.get_world_poses()[0][0]+ np.array([2, 0.5, 0.4]),
            current_joint_positions=my_left_ur10.get_joint_positions(),
            end_effector_offset=np.array([0.855, 0, -0.01]),
        )
        # action_2 = my_left_controller.forward(
        #     picking_position=left_diesel.get_world_poses()[0][0],
        #     placing_position=np.array([1.4, 1, 1]),
        #     current_joint_positions=my_left_ur10.get_joint_positions(),
        #     end_effector_offset=np.array([-0.5, 0, 0]),
        # )
        if my_left_controller.is_done() and not task_completed:
            print("done picking and placing")
            task_completed = True
        left_articulation_controller.apply_action(actions)
    # if args.test is True:
    #     break

# simulation_app.close()