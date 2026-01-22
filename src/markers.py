import cv2
import yaml

def center_crop(image, crop_size):
    h, w = image.shape[:2]

    crop_h, crop_w = crop_size
    center_h, center_w = h // 2, w // 2

    start_x = center_w - crop_w // 2
    start_y = center_h - crop_h // 2

    cropped_image = image[start_y:start_y + crop_h, start_x:start_x + crop_w]
    
    return cropped_image

def detect_aruco(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    normal = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    gray_eq = clahe.apply(normal)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    parameters.errorCorrectionRate = 1.0    # 더 많은 비트 오류 허용
    parameters.adaptiveThreshConstant = 7   # 이미지 품질이 좋으면 낮춰도 무방
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    corners, ids, _ = detector.detectMarkers(gray_eq)
    print("=====ARUCO DETECTION=====")
    print(ids)

    if len(corners) > 0:
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4,2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] -15), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
            print(f"[INFO] ArUco marker ID: {markerID}")

    return ids, corners

with open("/home/rokey/auto_oil_project/src/config.yaml", "r", encoding="utf-8") as f:
    data = yaml.safe_load(f)

customers = data["customers"]

def get_customer_by_aruco(aruco_id):
    for c in customers:
        if c["aruco_id"] == aruco_id:
            return c
    return None