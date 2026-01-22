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
import omni
import carb
import math
import numpy as np
from pxr import UsdLux, Sdf
from isaacsim.core.api import World
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import euler_angles_to_quat

class RMPFlowController(mg.MotionPolicyController):

    def __init__(
        self,
        name: str,
        robot_articulation: SingleArticulation,
        physics_dt: float = 1.0 / 60.0,
        attach_gripper: bool = False,
    ) -> None:

        if attach_gripper:
            self.rmp_flow_config = mg.interface_config_loader.load_supported_motion_policy_config(
                "UR10", "RMPflowSuction"
            )
        else:
            self.rmp_flow_config = mg.interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflow")
        self.rmp_flow = mg.lula.motion_policies.RmpFlow(**self.rmp_flow_config)

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)

        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        (
            self._default_position,
            self._default_orientation,
        ) = self._articulation_motion_policy._robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )

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
asset_path = "/home/rokey/auto_oil_project/assets/car.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Car")
# Car prim 생성 후
car = XFormPrim("/World/Car")
car.set_world_poses(positions=np.array([[0.0, 0.0, 0.0]]) / get_stage_units(), orientations=np.array([[1/math.sqrt(2),0,0,1/math.sqrt(2)]]))
oil_door = XFormPrim("/World/Car/World/ChassisRender/oildoor")
# cc = omni.usd.get_context()
# cc.open_stage(asset_path)
################################################################################################################################

asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
stage.DefinePrim("/World/left_base", "Xform")
stage.DefinePrim("/World/right_base", "Xform")
left_base = XFormPrim("/World/left_base")
right_base = XFormPrim("/World/right_base")
left_base.set_world_poses(np.array([[1.5, 1.3, 0]]))
right_base.set_world_poses(np.array([[1.3, -1.5, 0]]))
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

def move_point(robots,cspace_controller,goal_position: np.ndarray, end_effector_orientation: np.ndarray=np.array([0, np.pi/2, 0])) -> bool:
    # 목표 위치 배열의 차원을 (3,)으로 명확하게 지정
    goal_position = goal_position.squeeze()
    goal_position += np.array([0,0,0.04])
    end_effector_orientation = euler_angles_to_quat(end_effector_orientation)
    target_joint_positions = cspace_controller.forward(
        target_end_effector_position=goal_position, 
        target_end_effector_orientation=end_effector_orientation
    )
    robots.apply_action(target_joint_positions)
    current_joint_positions = robots.get_joint_positions()
    is_reached = np.all(np.abs(current_joint_positions[:7] - target_joint_positions.joint_positions) < 0.001)
    return is_reached

"""
left lpg gun(2.1, 0.9,0)
left diesel gun(1.8, 0.9,0)
left lpg gun(1.5, 0.9,0)
"""

# initialize the world
my_left_ur10.gripper.set_default_state(opened=True)
my_right_ur10.gripper.set_default_state(opened=True)

my_world.reset()
goal_reached = False
task_phase = 1
cspace_controller=RMPFlowController(name="my_ur10_cspace_controller", robot_articulation=my_left_ur10, attach_gripper=True)

while simulation_app.is_running():
    my_world.step()
    
    if task_phase == 1:
            print('----------------phase 1------------------')
            goal_reached = move_point(my_left_ur10, cspace_controller, np.array((2, 1.3, 1), dtype=np.float32))
            if goal_reached:
                cspace_controller.reset()
                task_phase = 2

    elif task_phase == 2:
        print('----------------phase 2------------------')
        print(left_diesel.get_world_poses())
        goal_reached = move_point(my_left_ur10,cspace_controller,
                                      np.array(left_lpg.get_world_poses()[0][:3], dtype=np.float32))
        if goal_reached:
            cspace_controller.reset()
            task_phase = 3

    elif task_phase == 3:
        print('----------------phase 3------------------')
        my_left_ur10.gripper.close()
        task_phase = 4

    elif task_phase == 4:
            print('----------------phase 4------------------')
            print(np.array(oil_door.get_world_poses()[1][:3]))
            goal_reached = move_point(my_left_ur10,cspace_controller,
                                      np.array((2, 0.5, 1), dtype=np.float32))
            if goal_reached:
                cspace_controller.reset()
                task_phase = 5

    elif task_phase == 5:
            print('----------------phase 4------------------')
            print(np.array(oil_door.get_world_poses()[1][:3]))
            goal_reached = move_point(my_left_ur10,cspace_controller,
                                      np.array((2, 0.9, 0.5), dtype=np.float32))
                                      #np.array(oil_door.get_world_poses()[0][:3], dtype=np.float32))
            if goal_reached:
                cspace_controller.reset()
                task_phase = 6

    elif task_phase == 6:
            print('----------------phase 4------------------')
            print(np.array(oil_door.get_world_poses()[1][:3]))
            goal_reached = move_point(my_left_ur10,cspace_controller,
                                      np.array((2.1, 0.9, 0.5), dtype=np.float32))
            if goal_reached:
                cspace_controller.reset()
                task_phase = 7
    elif task_phase == 7:
        print('----------------phase 3------------------')
        my_left_ur10.gripper.open()
        task_phase = 8