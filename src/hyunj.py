# main.py
from robot_env import RobotEnv, simulation_app
from car_env import CarEnv
import numpy as np
from isaacsim.sensors.camera import Camera
from markers import detect_aruco, get_customer_by_aruco
import cv2

'''
left lpg(0.9, 2.1)
left diesel(0.9, 1.8)
left gasoline(0.9, 1.5)

right lpg(0.9 -2.1)
right diesel(0.9, -1.8)
right gasoline(0.9, -1.5)

left arm(1.5, 1.3)
right arm(1.5, -1.3)
'''

def phase_logic(env, car_type):
    
    # 함수 내에서 전역 변수를 수정하기 위해 global 키워드 사용
    global car_phase, task_phase
    global direction, oil_type
    global controller, robot, y
    

    # 1. 자동차 이동 (PHASE 1) - **수정 부분 시작**
    if car_phase == 1:
        target_x = 0.0
        
        # car_type에 따라 다른 함수 호출
        if car_type == "car":
            arrived = env.car_env.move_car_vehicle(target_x_pos=target_x)
        elif car_type == "taxi":
            arrived = env.car_env.move_taxi_vehicle(target_x_pos=target_x)
        elif car_type == "benz":
            arrived = env.car_env.move_benz_vehicle(target_x_pos=target_x-3.5)
        else:
            # 예상치 못한 car_type이 들어온 경우 처리
            print(f"[ERROR] 알 수 없는 차량 유형: {car_type}")
            return
    
        # 자동차가 주유 구역에 도착
        if arrived:
            # 카메라 센싱 시작
            rgba = camera.get_rgba()
            rgb = (rgba[:, :, :3]).astype(np.uint8)
            cv2.imwrite(f"./{car_type}.png", rgb)

            ids, corners = detect_aruco(rgb)

            if ids is not None:
                for ID in ids.flatten():
                    info = get_customer_by_aruco(int(ID))
                    if info is not None:
                        # car_type = info["car_type"]
                        direction = info["oil_pose"]
                        oil_type = info["fuel_type"]
                        print("===================")
                        print(direction, oil_type)
                        print("===================")
            else:
                print("[WARN] No ArUco detected at frame")
            try:
                controller = getattr(env, f"{direction}_controller")
                robot = getattr(env, f"{direction}_robot")
                gun_prim = getattr(env, f"{direction}_{oil_type}")
            except AttributeError:
                print(f"[ERROR] '{direction}' 방향의 컨트롤러, 로봇 또는 주유 건({oil_type}) 객체를 찾을 수 없습니다.")
                return
            
            if direction == "left":
                if oil_type == "lpg":
                    y = 2.1
                elif oil_type == "diesel":
                    y = 1.8
                elif oil_type == "gasoline":
                    y =1.5
            elif direction == "right":
                if oil_type == "lpg":
                    y = -2.1
                elif oil_type == "diesel":
                    y = -1.8
                elif oil_type == "gasoline":
                    y =-1.5
            car_phase = 2
        return

    # 3. 로봇 작업 시작 (car_phase 2) (생략 - 변경 없음)
    elif car_phase == 2:
        
        # TASK 1 ~ TASK 7 로직... (변경 없음)
        if task_phase == 1:
            goal = np.array((1.5, 1.3 if direction == 'left' else -1.3, 0.5), np.float32)
            if env.move_point(direction, goal):
                controller.reset()
                task_phase = 2

        elif task_phase == 2:
            goal = np.array((0.9, y, 0.03), np.float32)
            if env.move_point(direction, goal):
                controller.reset()
                task_phase = 3

        elif task_phase == 3:
            robot.gripper.close()
            task_phase = 4

        elif task_phase == 4:
            goal = np.array((1.5, 1.3 if direction == 'left' else -1.3, 1), np.float32)
            if env.move_point(direction, goal):
                controller.reset()
                task_phase = 5

        elif task_phase == 5:
            goal = np.array((0.5, -1.8, 1), np.float32)
            if env.move_point(direction, goal):
                controller.reset()
                task_phase = 6

        elif task_phase == 6:
            
            goal = np.array((0.9, y, 0.5), np.float32)
            if env.move_point(direction, goal):
                controller.reset()
                task_phase = 7

        elif task_phase == 7:
            robot.gripper.open()
            car_phase = 3
        
        return

    # 4. 자동차 퇴장 (PHASE 3) - **수정 부분 시작**
    elif car_phase == 3:
        target_x = -20 
        
        # car_type에 따라 다른 함수 호출
        if car_type == 'car':
            arrived = env.car_env.move_car_vehicle(target_x_pos=target_x)
            arrived = env.car_env.move_taxi_vehicle(target_x_pos=target_x+25)
            arrived = env.car_env.move_benz_vehicle(target_x_pos=target_x+30)
        elif car_type == 'taxi':
            arrived = env.car_env.move_taxi_vehicle(target_x_pos=target_x+5)
            arrived = env.car_env.move_benz_vehicle(target_x_pos=target_x+25)
        elif car_type == 'benz':
            arrived = env.car_env.move_benz_vehicle(target_x_pos=target_x+10)
        else:
            print(f"[ERROR] 알 수 없는 차량 유형: {car_type}")
            return
        if arrived:
            car_phase = 4
            return
    
# CarEnv 생성 (stage는 RobotEnv 생성 후 할당)
temp_car_env = CarEnv(stage=None)

# RobotEnv 생성하고 car_env 전달
env = RobotEnv(car_env=temp_car_env)

# 이제 car_env의 stage를 RobotEnv가 만든 stage로 재할당
temp_car_env.stage = env.stage

temp_car_env.setup_car()

# default_world.usd 파일에 카메라 추가 (위치, 방향 확인)
camera = Camera(prim_path="/World/Camera", frequency=20, resolution=(256, 256))

env.world.reset()
camera.initialize()

camera.add_motion_vectors_to_frame()

# ... (env 초기화 및 전역 변수 정의) ...
car_phase = 1
task_phase = 1
phase =1
while simulation_app.is_running():
    env.world.step()
    
    if phase ==1:
        phase_logic(env, "car")
        if car_phase == 4 and task_phase ==7:
            phase = 2
            car_phase=1
            task_phase=1
        
    elif phase == 2:
        phase_logic(env, "taxi")
        if car_phase == 4 and task_phase ==7:
            phase = 3
            car_phase=1
            task_phase=1

    elif phase ==3:
        phase_logic(env, "benz")
        if car_phase == 4 and task_phase ==7:
            phase = 2
            car_phase=1
            task_phase=1