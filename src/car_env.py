# car_env.py
import math
import numpy as np
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from pxr import UsdGeom, Gf, UsdPhysics
import carb

# 카메라 위치 (0,0,3)
# QR은 차량 지붕 약간 앞부분
# 벤츠 조금 더 앞까지 기도록

class CarEnv:
    # 휠 조인트 및 상수 정의
    BODY_PATH = "/World/Car/World/ChassisRender"
    WHEEL_JOINT_PATHS = [
        "/World/Car/World/joints/front_left_joint",
        "/World/Car/World/joints/front_right_joint",
        "/World/Car/World/joints/back_left_joint",
        "/World/Car/World/joints/back_right_joint",
    ]
    
    taxi_BODY_PATH = "/World/Taxi/taxi11/World_001/_3914_Taxi_v2_L1/node_3914_Taxi_v2_L1"
    taxi_WHEEL_JOINT_PATHS = [  
        "/World/Taxi/taxi11/World_001/_3914_Taxi_v2_L1/joints/front_left_joint",
        "/World/Taxi/taxi11/World_001/_3914_Taxi_v2_L1/joints/front_right_joint",
        "/World/Taxi/taxi11/World_001/_3914_Taxi_v2_L1/joints/back_left_joint",
        "/World/Taxi/taxi11/World_001/_3914_Taxi_v2_L1/joints/back_right_joint",
    ]

    benz_BODY_PATH = "/World/Benz/World_001/uploads_files_2787791_Mercedes_Benz_GLS_580/Mercedes_Benz_GLS_580_ID3358"
    benz_WHEEL_JOINT_PATHS = [
        "/World/Benz/World_001/joints/front_left_joint",
        "/World/Benz/World_001/joints/front_right_joint",
        "/World/Benz/World_001/joints/back_left_joint",
        "/World/Benz/World_001/joints/back_right_joint",
    ]

    WHEEL_SPEED = 300.0  # 기본 이동 속도 (deg/s)
    STOP_DIST = 0.5      # 정지 임계값 (미터)

    def __init__(self, stage):
        """
        stage: World.stage 즉 USD Stage를 외부에서 받음
        """
        self.stage = stage
        self.wheel_drive_vel_attrs = []
        self.taxi_wheel_drive_vel_attrs = []
        self.benz_wheel_drive_vel_attrs = []

        #self._load_car()
        #self._setup_car_drive()
        
    def setup_car(self):
        """stage가 유효한지 확인된 후 호출됩니다."""
        if self.stage is None:
             raise Exception("CarEnv.setup_car()를 호출하기 전에 stage가 할당되지 않았습니다.")
        self._load_car()
        self._setup_car_drive()
        
    def _load_car(self):
        car_path = "/home/rokey/auto_oil_project/assets/car.usd"

        # /World/Car 에 로드
        add_reference_to_stage(
            usd_path=car_path,
            prim_path="/World/Car"
        )

        # 자동차 Prim 핸들
        self.car = XFormPrim("/World/Car")

        # 자동차 위치 기본 세팅 (main.py의 5.0m 위치로 변경했습니다. 필요 시 수정하세요)
        self.car.set_world_poses(
            positions=np.array([[25.0, 0.2, 0.0]]) / get_stage_units(),
        )
        self.car.set_local_scales(np.array([[1,1,0.8]]))

        # 자동차 내부 파트 예: 오일 도어
        self.car_oil_door = XFormPrim("/World/Car/World/ChassisRender/oildoor")
        
        # 바디 Xformable 핸들
        body_prim = self.stage.GetPrimAtPath(self.BODY_PATH)
        if not body_prim.IsValid():
            carb.log_error(f"Error: Body Prim not found at path: {self.BODY_PATH}")
            return
        self.body_xform = UsdGeom.Xformable(body_prim)

        taxi_path = "/home/rokey/auto_oil_project/assets/taxi.usd"

        # /World/Car 에 로드
        add_reference_to_stage(
            usd_path=taxi_path,
            prim_path="/World/Taxi"
        )

        # 자동차 Prim 핸들
        self.taxi = XFormPrim("/World/Taxi")

        # 자동차 위치 기본 세팅 (main.py의 5.0m 위치로 변경했습니다. 필요 시 수정하세요)
        self.taxi.set_world_poses(
            positions=np.array([[15.0, 0.0, 0.0]]) / get_stage_units(),
        )

        # 자동차 내부 파트 예: 오일 도어
        self.taxi_oil_door = XFormPrim("/World/Taxi/taxi11/World_001/_3914_Taxi_v2_L1/node_3914_Taxi_v2_L1/Capsule")
        
        # 바디 Xformable 핸들
        taxi_body_prim = self.stage.GetPrimAtPath(self.taxi_BODY_PATH)
        if not taxi_body_prim.IsValid():
            carb.log_error(f"Error: Body Prim not found at path: {self.taxi_BODY_PATH}")
            return
        self.taxi_body_xform = UsdGeom.Xformable(taxi_body_prim)
        

        benz_car_path = "/home/rokey/auto_oil_project/assets/benz.usd"

        # /World/Car 에 로드
        add_reference_to_stage(
            usd_path=benz_car_path,
            prim_path="/World/Benz"
        )

        # 자동차 Prim 핸들
        self.benz = XFormPrim("/World/Benz")

        # 자동차 위치 기본 세팅 (main.py의 5.0m 위치로 변경했습니다. 필요 시 수정하세요)
        self.benz.set_world_poses(
            positions=np.array([[3, 1.4, 0.2]]) / get_stage_units(),
        )
        self.benz.set_local_scales(np.array([[0.6,0.6,0.6]]))

        # 자동차 내부 파트 예: 오일 도어
        self.benz_oil_door = XFormPrim("/World/Car/World/ChassisRender/oildoor")
        
        # 바디 Xformable 핸들
        benz_body_prim = self.stage.GetPrimAtPath(self.benz_BODY_PATH)
        if not benz_body_prim.IsValid():
            carb.log_error(f"Error: Body Prim not found at path: {self.benz_BODY_PATH}")
            return
        self.benz_body_xform = UsdGeom.Xformable(benz_body_prim)
    def _setup_car_drive(self):
        """
        자동차 휠 조인트에 UsdPhysics.DriveAPI를 설정합니다.
        """
        for path in self.WHEEL_JOINT_PATHS:
            joint_prim = self.stage.GetPrimAtPath(path)
            if not joint_prim.IsValid():
                print(f"[WARN] joint prim not found: {path}")
                continue

            # DriveAPI 설정
            drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(0.0)
                drive.CreateMaxForceAttr().Set(1e6) # 충분한 최대 힘 설정

            # TargetVelocity 속성을 캐시 (이 속성을 통해 속도를 제어)
            vel_attr = drive.GetTargetVelocityAttr()
            if not vel_attr:
                vel_attr = drive.CreateTargetVelocityAttr(0.0)

            self.wheel_drive_vel_attrs.append((path, vel_attr))
        
        for path in self.taxi_WHEEL_JOINT_PATHS:
            joint_prim = self.stage.GetPrimAtPath(path)
            if not joint_prim.IsValid():
                print(f"[WARN] joint prim not found: {path}")
                continue

            drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(0.0)
                drive.CreateMaxForceAttr().Set(1e6)

            vel_attr = drive.GetTargetVelocityAttr()
            if not vel_attr:
                vel_attr = drive.CreateTargetVelocityAttr(0.0)

            self.taxi_wheel_drive_vel_attrs.append((path, vel_attr))

        for path in self.benz_WHEEL_JOINT_PATHS:
            joint_prim = self.stage.GetPrimAtPath(path)
            if not joint_prim.IsValid():
                print(f"[WARN] joint prim not found: {path}")
                continue

            drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(0.0)
                drive.CreateMaxForceAttr().Set(1e6)

            vel_attr = drive.GetTargetVelocityAttr()
            if not vel_attr:
                vel_attr = drive.CreateTargetVelocityAttr(0.0)

            self.benz_wheel_drive_vel_attrs.append((path, vel_attr))
##############################################################################################################################################
    def set_wheel_velocity(self, speed: float):
        """
        모든 휠의 각속도를 설정하여 자동차를 움직입니다.
        """
        for path, attr in self.wheel_drive_vel_attrs:
            if not attr:
                print(f"[WARN] no velocity attr cached for {path}")
                continue
            # Note: 속도는 로봇의 RMPFlow처럼 관절에 직접 작용합니다.

            attr.Set(float(speed))
    def set_taxi_wheel_velocity(self, speed: float):
        """
        모든 휠의 각속도를 설정하여 자동차를 움직입니다.
        """
        for path, attr in self.taxi_wheel_drive_vel_attrs:
            if not attr:
                print(f"[WARN] no velocity attr cached for {path}")
                continue
            # Note: 속도는 로봇의 RMPFlow처럼 관절에 직접 작용합니다.
            attr.Set(float(speed))
    def set_benz_wheel_velocity(self, speed: float):
        """
        모든 휠의 각속도를 설정하여 자동차를 움직입니다.
        """
        for path, attr in self.benz_wheel_drive_vel_attrs:
            if not attr:
                print(f"[WARN] no velocity attr cached for {path}")
                continue
            # Note: 속도는 로봇의 RMPFlow처럼 관절에 직접 작용합니다.
            attr.Set(float(speed))

    def get_body_world_pos(self) -> Gf.Vec3d:
        """
        자동차 바디의 월드 좌표를 반환합니다.
        """
        # 현재 시뮬레이션 시간(0.0)에서의 변환을 계산
        mat = self.body_xform.ComputeLocalToWorldTransform(0.0)
        return mat.ExtractTranslation()
    
    def get_taxi_body_world_pos(self) -> Gf.Vec3d:
        """
        자동차 바디의 월드 좌표를 반환합니다.
        """
        # 현재 시뮬레이션 시간(0.0)에서의 변환을 계산
        mat = self.taxi_body_xform.ComputeLocalToWorldTransform(0.0)
        return mat.ExtractTranslation()
    
    def get_benz_body_world_pos(self) -> Gf.Vec3d:
        """
        자동차 바디의 월드 좌표를 반환합니다.
        """
        # 현재 시뮬레이션 시간(0.0)에서의 변환을 계산
        mat = self.benz_body_xform.ComputeLocalToWorldTransform(0.0)
        return mat.ExtractTranslation()
        
    def move_car_vehicle(self, target_x_pos: float, speed: float = None):
        """
        자동차를 월드 X축 방향으로 목표 X좌표까지 이동시키고 정지합니다.
        이동이 완료되면 True를 반환합니다.
        """
        current_pos = self.get_body_world_pos()
        dx = target_x_pos - current_pos[0]

        speed = speed if speed is not None else self.WHEEL_SPEED

        if abs(dx) > self.STOP_DIST:
            # dx > 0 → 목표가 오른쪽, dx < 0 → 목표가 왼쪽
            # Isaac Sim PhysX의 기본 관절 회전 방향에 따라 -1.0과 1.0을 결정해야 합니다.
            # main.py의 로직을 따릅니다: dx > 0(오른쪽)일 때 -1.0 * speed (역방향 회전)
            direction = -1.0 if dx > 0 else 1.0
            self.set_wheel_velocity(direction * speed)
            return False  # 아직 이동 중

        # 도착
        self.set_wheel_velocity(0.0)
        return True
    
    def move_taxi_vehicle(self, target_x_pos: float, speed: float = None):
        """
        자동차를 월드 X축 방향으로 목표 X좌표까지 이동시키고 정지합니다.
        이동이 완료되면 True를 반환합니다.
        """
        current_pos = self.get_taxi_body_world_pos()
        dx = target_x_pos - current_pos[0]
        speed = speed if speed is not None else self.WHEEL_SPEED

        if abs(dx) > self.STOP_DIST:
            # dx > 0 → 목표가 오른쪽, dx < 0 → 목표가 왼쪽
            # Isaac Sim PhysX의 기본 관절 회전 방향에 따라 -1.0과 1.0을 결정해야 합니다.
            # main.py의 로직을 따릅니다: dx > 0(오른쪽)일 때 -1.0 * speed (역방향 회전)
            direction = -1.0 if dx > 0 else 1.0
            self.set_taxi_wheel_velocity(direction * speed)
            return False  # 아직 이동 중

        # 도착
        self.set_taxi_wheel_velocity(0.0)
        return True
    
    def move_benz_vehicle(self, target_x_pos: float, speed: float = None):
        """
        자동차를 월드 X축 방향으로 목표 X좌표까지 이동시키고 정지합니다.
        이동이 완료되면 True를 반환합니다.
        """
        current_pos = self.get_benz_body_world_pos()
        dx = target_x_pos - current_pos[0]
        speed = speed if speed is not None else self.WHEEL_SPEED

        if abs(dx) > self.STOP_DIST:
            # dx > 0 → 목표가 오른쪽, dx < 0 → 목표가 왼쪽
            # Isaac Sim PhysX의 기본 관절 회전 방향에 따라 -1.0과 1.0을 결정해야 합니다.
            # main.py의 로직을 따릅니다: dx > 0(오른쪽)일 때 -1.0 * speed (역방향 회전)
            direction = -1.0 if dx > 0 else 1.0
            self.set_benz_wheel_velocity(direction * speed/2)
            return False  # 아직 이동 중

        # 도착
        self.set_benz_wheel_velocity(0.0)
        return True