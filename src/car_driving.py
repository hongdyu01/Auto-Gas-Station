from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
})

import omni
import carb
from carb.input import MouseInput

# # # 마우스/입력 인터페이스 얻기
input_iface = carb.input.acquire_input_interface()
mouse = omni.appwindow.get_default_app_window().get_mouse()
from pxr import UsdGeom, Gf, UsdPhysics
import omni.usd
import omni.kit.app
import omni.timeline
import os

ASSET_ROOT = "/home/rokey/auto_oil_project/assets/"

CAR_PATH = ASSET_ROOT + "car.usd"
BODY_PATH = "/root/World/ChassisRender"

WHEEL_JOINT_PATHS = [
    "/root/World/joints/front_left_joint",
    "/root/World/joints/front_right_joint",
    "/root/World/joints/back_left_joint",
    "/root/World/joints/back_right_joint",
]

TAXI_PATH = ASSET_ROOT + "taxi.usd"
taxi_BODY_PATH = "/World/taxi11/taxi11/World_001/_3914_Taxi_v2_L1/node_3914_Taxi_v2_L1"

taxi_WHEEL_JOINT_PATHS = [  
    "/World/taxi11/taxi11/World_001/_3914_Taxi_v2_L1/joints/front_left_joint",
    "/World/taxi11/taxi11/World_001/_3914_Taxi_v2_L1/joints/front_right_joint",
    "/World/taxi11/taxi11/World_001/_3914_Taxi_v2_L1/joints/back_left_joint",
    "/World/taxi11/taxi11/World_001/_3914_Taxi_v2_L1/joints/back_right_joint",
]

benz_path = ASSET_ROOT + "benz.usd"
# 🔹 벤츠 차체
benz_BODY_PATH = "/root/World_001/World_001/uploads_files_2787791_Mercedes_Benz_GLS_580/Mercedes_Benz_GLS_580_ID3358"

# 🔹 벤츠 바퀴 조인트
benz_WHEEL_JOINT_PATHS = [
    "/root/World_001/World_001/joints/front_left_joint",
    "/root/World_001/World_001/joints/front_right_joint",
    "/root/World_001/World_001/joints/back_left_joint",
    "/root/World_001/World_001/joints/back_right_joint",
]


# 🔹 1차 목표: 중간 정지 지점
WAIT_POS = Gf.Vec3d(-3.0, 0.0, 0.0) # 주유 대기 위치
 
MID_STOP_POS = Gf.Vec3d(-10.0, 0.0, 0.0)   # 주유 위치

# 🔹 2차 목표: 최종 목적지
TARGET_POS = Gf.Vec3d(-200.0, 0.0, 0.0)    # 나가는 위치

WHEEL_SPEED = 150.0
STOP_DIST   = 0.5   # 정지 임계값

world = omni.usd.get_context()
world.open_stage(CAR_PATH)

def spawn_taxi_instance(stage, taxi_path: str, prim_path: str, position):
    """
    stage 안에 usd_path를 참조하는 차량 인스턴스를 prim_path에 만들고,
    그 Prim을 world 기준으로 position(x, y, z)에 배치한다.
    """
    # 1) 빈 Xform Prim 하나 정의
    taxi_root = stage.DefinePrim(prim_path, "Xform")
    
    # 2) 그 Prim에 usd 파일을 Reference로 붙이기
    taxi_root.GetReferences().AddReference(taxi_path)

    # 3) 위치 지정
    xform_api = UsdGeom.XformCommonAPI(taxi_root)
    xform_api.SetTranslate(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))

    print(f"[INFO] Spawned car from {taxi_path} at {position} as {prim_path}")
    return taxi_root

def spawn_benz_instance(stage, benz_path: str, prim_path: str, position):
 
    # 1) 빈 Xform Prim 하나 정의
    benz_root = stage.DefinePrim(prim_path, "Xform")
    
    # 2) 그 Prim에 usd 파일을 Reference로 붙이기
    benz_root.GetReferences().AddReference(benz_path)

    # 3) 위치 지정
    xform_api = UsdGeom.XformCommonAPI(benz_root)
    xform_api.SetTranslate(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))

    print(f"[INFO] Spawned car from {benz_path} at {position} as {prim_path}")
    return benz_root


# 로딩 기다리기
for _ in range(120):
    simulation_app.update()

stage = world.get_stage()
if stage is None:
    print(f"[ERROR] Failed to open stage from: {CAR_PATH}")
    simulation_app.close()
    raise SystemExit(1)

# 택시 호출
spawn_taxi_instance(
    stage,
    TAXI_PATH,                # 불러올 택시 usd 경로
    "/World/taxi11",          # 이 스테이지 안에서의 루트 prim 경로
    (5.0, -0.5, 0.0),         # 원하는 생성 위치 (x, y, z)
)

spawn_benz_instance(
    stage,
    benz_path,                # 불러올 택시 usd 경로
    "/root/World_001",          # 이 스테이지 안에서의 루트 prim 경로
    (10.0, 2.0, 0.0),         # 원하는 생성 위치 (x, y, z)
)


# 🔹 타임라인 재생 시작
timeline = omni.timeline.get_timeline_interface()
timeline.play()

# =========================
# ② Drive 핸들 준비 (이하 동일)
# =========================

# ------------------------------------------- car
wheel_drive_vel_attrs = []

for path in WHEEL_JOINT_PATHS:
    joint_prim = stage.GetPrimAtPath(path)
    if not joint_prim.IsValid():
        print(f"[WARN] joint prim not found: {path}")
        continue

    drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
    if not drive:
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.0)
        # drive.CreateMaxForceAttr().Set(0.0)
        drive.CreateMaxForceAttr().Set(1e6)

    vel_attr = drive.GetTargetVelocityAttr()
    if not vel_attr:
        vel_attr = drive.CreateTargetVelocityAttr(0.0)

    wheel_drive_vel_attrs.append((path, vel_attr))

#---------------------------------- 택시

taxi_wheel_drive_vel_attrs = []

for path in taxi_WHEEL_JOINT_PATHS:
    joint_prim = stage.GetPrimAtPath(path)
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

    taxi_wheel_drive_vel_attrs.append((path, vel_attr))

#---------------------------------- benz

benz_wheel_drive_vel_attrs = []

for path in benz_WHEEL_JOINT_PATHS:
    joint_prim = stage.GetPrimAtPath(path)
    if not joint_prim.IsValid():
        print(f"[WARN] joint prim not found: {path}")
        continue

    drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
    if not drive:
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.0)
        # drive.CreateMaxForceAttr().Set(0.0)
        drive.CreateMaxForceAttr().Set(1e6)

    vel_attr = drive.GetTargetVelocityAttr()
    if not vel_attr:
        vel_attr = drive.CreateTargetVelocityAttr(0.0)

    benz_wheel_drive_vel_attrs.append((path, vel_attr))

# print("[INFO] Drive targetVelocity attrs prepared:",
#       [p for p, _ in wheel_drive_vel_attrs])

# =========================
# ③ 차체 Prim 찾기 (이하 동일)
# =========================

body_prim = stage.GetPrimAtPath(BODY_PATH)
body_xform = UsdGeom.Xformable(body_prim)

# 🔹 택시 차체
taxi_body_xform = None  # ✅ 먼저 기본값으로 선언

taxi_body_prim = stage.GetPrimAtPath(taxi_BODY_PATH)
if not taxi_body_prim.IsValid():
    print(f"[ERROR] taxi BODY_PATH Prim not found: {taxi_BODY_PATH}")
    print("[TIP] Isaac Sim Stage 창에서 택시 차체 Prim 우클릭 → Copy Path 해서 taxi_BODY_PATH에 그대로 넣어줘.")
else:
    taxi_body_xform = UsdGeom.Xformable(taxi_body_prim)

# 🔹 벤츠 차체 (원하면 나중에 사용)
benz_body_prim = stage.GetPrimAtPath(benz_BODY_PATH)
if not benz_body_prim.IsValid():
    print(f"[WARN] benz BODY_PATH Prim not found: {benz_BODY_PATH}")
    benz_body_xform = None
else:
    benz_body_xform = UsdGeom.Xformable(benz_body_prim)

# =========================
# ④ 유틸 함수 (이하 동일)
# =========================

def set_wheel_velocity(speed: float):
    for path, attr in wheel_drive_vel_attrs:
        if not attr:
            print(f"[WARN] no velocity attr cached for {path}")
            continue
        attr.Set(float(speed))

# ------------------------------- taxi

def set_taxi_velocity(speed: float):
    for path, attr in taxi_wheel_drive_vel_attrs:
        if attr:
            attr.Set(float(speed))

# ------------------------------- benz

def set_benz_velocity(speed: float):
    for path, attr in benz_wheel_drive_vel_attrs:
        if attr:
            attr.Set(float(speed))

def get_body_world_pos() -> Gf.Vec3d:
    mat = body_xform.ComputeLocalToWorldTransform(0.0)
    return mat.ExtractTranslation()

def get_taxi_world_pos() -> Gf.Vec3d:
    if taxi_body_xform is None:
        return Gf.Vec3d(0.0, 0.0, 0.0)
    mat = taxi_body_xform.ComputeLocalToWorldTransform(0.0)
    return mat.ExtractTranslation()


def get_benz_world_pos() -> Gf.Vec3d:
    if benz_body_xform is None:
        return Gf.Vec3d(0.0, 0.0, 0.0)
    mat = benz_body_xform.ComputeLocalToWorldTransform(0.0)
    return mat.ExtractTranslation()

# --------------------------
#  공통 설정
# --------------------------
max_frames = 10000
STOP_DIST = 0.5

# --------------------------
#  Phase / 클릭 상태 변수
# --------------------------
car_phase  = 1   # 1: MID로 이동, 2: MID에서 대기, 3: TARGET 이동, 4: 완료
taxi_phase = 0   # 0: 대기, 1: WAIT 이동, 2: WAIT 대기, 3: MID 이동, 4: MID 대기, 5: TARGET 이동, 6: 완료
benz_phase = 0   # 0: 대기, 1: WAIT 이동, 2: WAIT 대기, 3: MID 이동, 4: 완료

click_count = 0  # 0: 아직 클릭 없음, 1: 첫 클릭 처리, 2: 두 번째 클릭 처리
mouse_prev_down = False

# --------------------------
#  이동 보조 함수
# --------------------------
def move_vehicle(current_pos, target_pos, set_vel_func, speed=WHEEL_SPEED):
    dx = target_pos[0] - current_pos[0]

    if abs(dx) > STOP_DIST:
        # dx > 0 → 목표가 오른쪽, dx < 0 → 목표가 왼쪽
        direction = -1.0 if dx > 0 else 1.0
        set_vel_func(direction * speed)
        return False  # 아직 이동 중

    # 도착
    set_vel_func(0.0)
    return True


# ==========================
#   메인 시뮬레이션 루프
# ==========================
for frame in range(max_frames):
    # --- 위치 읽기 ---
    car_pos  = get_body_world_pos()
    taxi_pos = get_taxi_world_pos()
    benz_pos = get_benz_world_pos()

    # --- 마우스 클릭 edge 검출 ---
    mouse_down  = input_iface.get_mouse_value(mouse, MouseInput.LEFT_BUTTON) > 0.5
    mouse_click = mouse_down and not mouse_prev_down  # 이번 프레임에 새로 눌렸을 때만 True
    mouse_prev_down = mouse_down

    # =======================
    # ① car 제어
    # =======================
    if car_phase == 1:
        # car → MID_STOP_POS (자동)
        arrived = move_vehicle(car_pos, MID_STOP_POS, set_wheel_velocity)
        if arrived:
            car_phase = 2         # MID에서 대기
            taxi_phase = 1        # taxi → WAIT_POS 출발

    elif car_phase == 2:
        # MID_STOP_POS에서 대기 (첫 클릭까지)
        set_wheel_velocity(0.0)

    elif car_phase == 3:
        # car → TARGET_POS (첫 클릭 이후)
        arrived = move_vehicle(car_pos, TARGET_POS, set_wheel_velocity)
        if arrived:
            car_phase = 4         # 최종 도착

    elif car_phase >= 4:
        # TARGET 도착 후 정지 유지
        set_wheel_velocity(0.0)

    # =======================
    # ② taxi 제어
    # =======================
    if taxi_phase == 0:
        # 아직 대기
        set_taxi_velocity(0.0)

    elif taxi_phase == 1:
        # taxi → WAIT_POS (car가 MID 도착한 후 자동 시작)
        arrived = move_vehicle(taxi_pos, WAIT_POS, set_taxi_velocity)
        if arrived:
            taxi_phase = 2    # WAIT에서 대기

    elif taxi_phase == 2:
        # WAIT_POS에서 대기 (첫 클릭까지)
        set_taxi_velocity(0.0)

    elif taxi_phase == 3:
        # taxi → MID_STOP_POS (첫 클릭 이후)
        arrived = move_vehicle(taxi_pos, MID_STOP_POS, set_taxi_velocity)
        if arrived:
            taxi_phase = 4    # MID에서 대기 (두 번째 클릭까지)

    elif taxi_phase == 4:
        # MID_STOP_POS에서 대기
        set_taxi_velocity(0.0)

    elif taxi_phase == 5:
        # taxi → TARGET_POS (두 번째 클릭 이후)
        arrived = move_vehicle(taxi_pos, TARGET_POS, set_taxi_velocity)
        if arrived:
            taxi_phase = 6    # 완료

    elif taxi_phase >= 6:
        set_taxi_velocity(0.0)

    # =======================
    # ③ benz 제어
    # =======================
    if benz_phase == 0:
        set_benz_velocity(0.0)

    elif benz_phase == 1:
        # benz → WAIT_POS (첫 클릭 이후)
        arrived = move_vehicle(benz_pos, WAIT_POS, set_benz_velocity)
        if arrived:
            benz_phase = 2    # WAIT에서 대기 (두 번째 클릭까지)

    elif benz_phase == 2:
        # WAIT_POS에서 대기
        set_benz_velocity(0.0)

    elif benz_phase == 3:
        # benz → MID_STOP_POS (두 번째 클릭 이후)
        arrived = move_vehicle(benz_pos, MID_STOP_POS, set_benz_velocity)
        if arrived:
            benz_phase = 4    # 완료

    elif benz_phase == 4:
        set_benz_velocity(0.0)

    elif benz_phase == 5:
        # benz → TARGET_POS (세 번째 클릭 이후)
        arrived = move_vehicle(benz_pos, TARGET_POS, set_benz_velocity)
        if arrived:
            benz_phase = 6    # 완료

    elif benz_phase >= 6:
        set_benz_velocity(0.0)
    # =======================
    # ④ 클릭에 따른 단계 전환
    # =======================
    if mouse_click:
        # ----- 첫 번째 클릭 -----
        # car: TARGET_POS, taxi: MID_STOP_POS, benz: WAIT_POS
        if click_count == 0:
            # car는 MID에 도착해서 대기 중이어야 함
            if car_phase == 2 and taxi_phase == 2:
                click_count = 1
                car_phase  = 3   # car → TARGET_POS
                taxi_phase = 3   # taxi → MID_STOP_POS
                benz_phase = 1   # benz → WAIT_POS

        # ----- 두 번째 클릭 -----
        # taxi: TARGET_POS, benz: MID_STOP_POS
        elif click_count == 1:
            # taxi는 MID에 도착해서 대기 중, benz는 WAIT에 도착해서 대기 중이어야 함
            if taxi_phase == 4 and benz_phase == 2:
                click_count = 2
                taxi_phase = 5   # taxi → TARGET_POS
                benz_phase = 3   # benz → MID_STOP_POS

        elif click_count == 2:
            # taxi는 MID에 도착해서 대기 중, benz는 WAIT에 도착해서 대기 중이어야 함
            if benz_phase == 4:
                click_count = 3
                benz_phase = 5   # benz → MID_STOP_POS

    simulation_app.update()

# 시뮬레이션 유지
while True:
    simulation_app.update()

        
        
            
        #     else:
        #         self._brown_cube_position, _ = self.cube.get_world_pose()
        #         self.task_phase = 3
        # elif self.task_phase == 3: