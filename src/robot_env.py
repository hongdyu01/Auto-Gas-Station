# robot_env.py
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import math
import numpy as np
from pxr import UsdLux, Sdf
from isaacsim.core.api import World
from isaacsim.core.prims import XFormPrim, SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.utils.rotations import euler_angles_to_quat


class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name, robot_articulation, physics_dt=1/60, attach_gripper=False):
        config = mg.interface_config_loader.load_supported_motion_policy_config(
            "UR10",
            "RMPflowSuction" if attach_gripper else "RMPflow"
        )
        self.rmp_flow = mg.lula.motion_policies.RmpFlow(**config)
        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)
        super().__init__(name=name, articulation_motion_policy=self.articulation_rmp)

        self._default_position, self._default_orientation = \
            self._articulation_motion_policy._robot_articulation.get_world_pose()

        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation
        )

    def reset(self):
        super().reset()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation
        )


class RobotEnv:
    def __init__(self, car_env=None):
        self.world = World(stage_units_in_meters=1.0)
        self.stage = self.world.stage

        # default world load
        asset_path = "/home/rokey/auto_oil_project/assets/default_world.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World")

        # Light override
        for prim in self.stage.Traverse():
            if prim.GetTypeName() in ["DistantLight", "SphereLight", "RectLight", "CylinderLight", "DiskLight"]:
                self.stage.RemovePrim(prim.GetPath())

        light = UsdLux.DistantLight.Define(self.stage, Sdf.Path("/World/defaultLight"))
        light.CreateIntensityAttr(1000)

        # 자동차 환경 받기
        self.car_env = car_env

        # 로봇 로드
        robot_path = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        self._setup_robots(robot_path)

        # Controller
        self.right_controller = RMPFlowController(
            name="right_ur10_cspace",
            robot_articulation=self.right_robot,
            attach_gripper=True
        )
        self.left_controller = RMPFlowController(
            name="left_ur10_cspace",
            robot_articulation=self.left_robot,
            attach_gripper=True
        )
        self.left_gasoline = XFormPrim("/World/left_gasoline_gun")
        self.left_diesel = XFormPrim("/World/left_diesel_gun")
        self.left_lpg = XFormPrim("/World/left_lpg_gun")
        self.right_gasoline = XFormPrim("/World/right_gasoline_gun")
        self.right_diesel = XFormPrim("/World/right_diesel_gun")
        self.right_lpg = XFormPrim("/World/right_lpg_gun")
        

    def _setup_robots(self, robot_path):

        stage = self.stage
        stage.DefinePrim("/World/left_base", "Xform")
        stage.DefinePrim("/World/right_base", "Xform")

        left_base = XFormPrim("/World/left_base")
        right_base = XFormPrim("/World/right_base")

        rotation_180_quat = euler_angles_to_quat(np.array([0, 0, -np.pi/2]))
        rotation_180_quat_2d = np.expand_dims(rotation_180_quat, axis=0)

        left_base.set_world_poses(np.array([[1.5, 1.3, 0]]))
        right_base.set_world_poses(
            positions=np.array([[1.5, -1.3, 0]]),
            orientations=rotation_180_quat_2d # 쿼터니언 적용
        )

        add_reference_to_stage(robot_path, "/World/left_base/left_ur10")
        add_reference_to_stage(robot_path, "/World/right_base/right_ur10")

        left_prim = stage.GetPrimAtPath("/World/left_base/left_ur10")
        right_prim = stage.GetPrimAtPath("/World/right_base/right_ur10")
        left_prim.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")
        right_prim.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")

        left_gripper = SurfaceGripper(
            "/World/left_base/left_ur10/ee_link",
            "/World/left_base/left_ur10/ee_link/SurfaceGripper"
        )
        right_gripper = SurfaceGripper(
            "/World/right_base/right_ur10/ee_link",
            "/World/right_base/right_ur10/ee_link/SurfaceGripper"
        )

        self.left_robot = self.world.scene.add(
            SingleManipulator(
                prim_path="/World/left_base/left_ur10",
                name="left_ur10",
                end_effector_prim_path="/World/left_base/left_ur10/ee_link",
                gripper=left_gripper
            )
        )

        self.right_robot = self.world.scene.add(
            SingleManipulator(
                prim_path="/World/right_base/right_ur10",
                name="right_ur10",
                end_effector_prim_path="/World/right_base/right_ur10/ee_link",
                gripper=right_gripper
            )
        )

    def move_point(self, direction, goal, orientation=np.array([0, np.pi/2, 0])):
        """
        주어진 방향(left/right)의 로봇을 목표 위치로 이동시킵니다.
        
        direction: 'left' 또는 'right'
        goal: 목표 End-Effector 위치 (3D 벡터)
        orientation: 목표 End-Effector 회전 (오일러 각)
        """
        
        # 1. 동적 컨트롤러 및 로봇 객체 가져오기
        try:
            # self.left_controller 또는 self.right_controller
            controller = getattr(self, f"{direction}_controller")
            # self.left_robot 또는 self.right_robot
            robot = getattr(self, f"{direction}_robot")
        except AttributeError:
            print(f"[ERROR] '{direction}' 방향의 컨트롤러 또는 로봇 객체를 찾을 수 없습니다.")
            return False

        # 2. 목표 위치 및 방향 설정 (기존 로직 유지)
        # 0.02m 높이 보정
        goal = goal.squeeze() + np.array([0, 0, 0.02]) 
        quat = euler_angles_to_quat(orientation)

        # 3. RMPFlow를 사용하여 관절 목표 계산
        target = controller.forward(
            target_end_effector_position=goal,
            target_end_effector_orientation=quat
        )
        
        # 4. 로봇에 액션 적용
        robot.apply_action(target)

        # 5. 도착 여부 확인 및 반환
        curr = robot.get_joint_positions()
        # 주의: UR10은 6개의 관절을 가지지만, get_joint_positions은 보통 7개(기저 포함)를 반환할 수 있으므로, 
        # RMPFlow가 제어하는 관절 수에 맞춰 인덱싱해야 합니다. (기존 코드와 일관성 유지를 위해 curr[:7] 유지)
        return np.all(np.abs(curr[:7] - target.joint_positions) < 1e-3)

    def insert(self, direction, goal, orientation=np.array([0, np.pi, 0])):
        """
        주어진 방향(left/right)의 로봇을 목표 위치로 이동시킵니다.
        
        direction: 'left' 또는 'right'
        goal: 목표 End-Effector 위치 (3D 벡터)
        orientation: 목표 End-Effector 회전 (오일러 각)
        """
        
        # 1. 동적 컨트롤러 및 로봇 객체 가져오기
        try:
            # self.left_controller 또는 self.right_controller
            controller = getattr(self, f"{direction}_controller")
            # self.left_robot 또는 self.right_robot
            robot = getattr(self, f"{direction}_robot")
        except AttributeError:
            print(f"[ERROR] '{direction}' 방향의 컨트롤러 또는 로봇 객체를 찾을 수 없습니다.")
            return False

        # 2. 목표 위치 및 방향 설정 (기존 로직 유지)
        # 0.02m 높이 보정
        goal = goal.squeeze() + np.array([0, 0, 0.02]) 
        quat = euler_angles_to_quat(orientation)

        # 3. RMPFlow를 사용하여 관절 목표 계산
        target = controller.forward(
            target_end_effector_position=goal,
            target_end_effector_orientation=quat
        )
        
        # 4. 로봇에 액션 적용
        robot.apply_action(target)

        # 5. 도착 여부 확인 및 반환
        curr = robot.get_joint_positions()
        # 주의: UR10은 6개의 관절을 가지지만, get_joint_positions은 보통 7개(기저 포함)를 반환할 수 있으므로, 
        # RMPFlow가 제어하는 관절 수에 맞춰 인덱싱해야 합니다. (기존 코드와 일관성 유지를 위해 curr[:7] 유지)
        return np.all(np.abs(curr[:7] - target.joint_positions) < 1e-3)