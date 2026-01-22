from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import sys
import carb
import math
import numpy as np
from pxr import Usd, UsdLux, Sdf
from isaacsim.core.api import World
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from omni.isaac.motion_generation import LulaKinematicsSolver, ArticulationKinematicsSolver, interface_config_loader
from isaacsim.robot.manipulators.examples.universal_robots.controllers.pick_place_controller import PickPlaceController

####################################################################
# Asset Root Path
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)
asset_path = "/home/rokey/auto_oil_project/assets/default_world.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World")

####################################################################
# Light
stage = my_world.stage
for prim in stage.Traverse():
    if prim.GetTypeName() in ["DistantLight", "SphereLight", "RectLight", "CylinderLight", "DiskLight"]:
        override_prim = stage.OverridePrim(prim.GetPath())
        stage.RemovePrim(prim.GetPath())

light_prim = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/defaultLight"))
light_prim.CreateIntensityAttr(1000)
light_prim.CreateAngleAttr(0.53)

####################################################################
# Gun positions
left_diesel = XFormPrim("/World/left_diesel_gun")

####################################################################
# Add UR10 Robots
asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
stage.DefinePrim("/World/left_base", "Xform")
left_base = XFormPrim("/World/left_base")
left_base.set_world_poses(np.array([[2.5, 0, 0]]))
left_base.set_local_scales(np.array([[2, 2, 2]]))

left_ur10 = add_reference_to_stage(usd_path=asset_path, prim_path="/World/left_base/left_ur10")
left_ur10.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")

left_gripper = SurfaceGripper(
    end_effector_prim_path="/World/left_base/left_ur10/ee_link",
    surface_gripper_path="/World/left_base/left_ur10/ee_link/SurfaceGripper"
)
my_left_ur10 = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/left_base/left_ur10",
        name="left_ur10",
        end_effector_prim_path="/World/left_base/left_ur10/ee_link",
        gripper=left_gripper,
    )
)

my_left_ur10.gripper.set_default_state(opened=True)
my_world.reset()

####################################################################
# IK Solver 초기화
# Lula config 로드
config = interface_config_loader.load_supported_lula_kinematics_solver_config("UR10")
lula_solver = LulaKinematicsSolver(**config)
art_kine_solver = ArticulationKinematicsSolver(my_left_ur10, lula_solver, "ee_link")  # end-effector frame

####################################################################
# Waypoints 설정
pick_pos = left_diesel.get_world_poses()[0][0]
waypoints = [
    pick_pos + np.array([0.0, 0.0, 0.25]),  # 위로 25cm
    np.array([1.4, -2.0, 1.0]),            # 중간 경유
    np.array([1.4, -1.0, 1.0])             # placing
]
current_target_index = 0

left_articulation_controller = my_left_ur10.get_articulation_controller()

####################################################################
# Main loop
while simulation_app.is_running():
    my_world.step(render=True)

    if current_target_index >= len(waypoints):
        continue

    current_joints = my_left_ur10.get_joint_positions() 
    target_pos = waypoints[current_target_index]

    # IK 계산
    success, joint_targets = art_kine_solver.compute_inverse_kinematics(
        target_position=target_pos, 
        target_orientation=None
    )

    if success:
        # IK가 성공했을 때만! joint_targets는 유효한 관절 위치 배열(numpy array)이 됩니다.
        try:
            actions = ArticulationAction(joint_positions=joint_targets)
            left_articulation_controller.apply_action(actions)
        except TypeError as e:
            # IK 성공 시에도 TypeError가 발생하면 디버깅 정보 출력
            print(f"IK 성공 후 TypeError 발생: {e}")
            print(f"joint_targets 타입: {type(joint_targets)}, 값: {joint_targets}")
            continue

        # 도착 여부 확인 로직
        ee_pos = my_left_ur10.get_end_effector_position()
        if np.linalg.norm(target_pos - ee_pos) < 0.03:
            print(f"{current_target_index}번 waypoint 도착")
            current_target_index += 1
    else:
        # IK 실패 (success=False). joint_targets는 무시됩니다.
        print(f"IK 실패 - 목표 위치: {target_pos}")