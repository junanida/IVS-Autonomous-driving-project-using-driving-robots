import threading
from flask import Flask, Response
import cv2
from pinkylib import Camera, Motor, Ultrasonic   # ← Ultrasonic 추가
from pinky_lcd import LCD
from PIL import Image, ImageDraw
import numpy as np
import time
from ultralytics import YOLO
import numpy as np
from picamera2 import Picamera2

app = Flask(__name__)

cam = Camera()
cam.start()
motor = Motor()
lcd = LCD()
ultra = Ultrasonic()     # ← 추가!
motor.enable_motor()
motor.start_motor()

img_width, img_height = 320, 240
latest_frame = None

traffic_light_color = "UNKNOWN"  # 전역 상태 변수

def traffic_light_detector():
    global traffic_light_color
    model = YOLO("best.pt")

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                continue

            # 프레임 전처리 (YOLO 모델에 맞게 크기 조정 가능)
            resized_frame = cv2.resize(frame, (640, 480))

            # YOLO 예측
            results = model.predict(resized_frame, imgsz=640, conf=0.4)[0]

            detected = False
            for box in results.boxes:
                cls_id = int(box.cls[0])
                class_name = results.names[cls_id]
                if class_name != "traffic light":
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # 🔸 너무 작은 신호등은 무시
                if (x2 - x1) < 20 or (y2 - y1) < 20:
                    continue

                roi = resized_frame[y1:y2, x1:x2]
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                h, w = hsv.shape[:2]

                left  = hsv[:, 0:w//3]
                mid   = hsv[:, w//3:2*w//3]
                right = hsv[:, 2*w//3:]

                v_values = [np.mean(left[:, :, 2]), np.mean(mid[:, :, 2]), np.mean(right[:, :, 2])]
                max_idx = np.argmax(v_values)

                if max(v_values) < 60:
                    traffic_light_color = "UNKNOWN"
                else:
                    traffic_light_color = ["GREEN", "YELLOW", "RED"][max_idx]

                detected = True
                break  # 첫 번째 신호등만 판단

            if not detected:
                traffic_light_color = "UNKNOWN"

            time.sleep(0.1)

    except Exception as e:
        print(f"[YOLO ERROR] {e}")

def generate():
    global latest_frame
    while True:
        frame = cam.get_frame() if latest_frame is None else latest_frame
        if frame is None:
            continue
        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

@app.route('/')
def index():
    return "<h2>영상 스트리밍 중... <a href='/video_feed'>/video_feed</a>로 이동</h2>"

@app.route('/video_feed')
def video_feed():
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def flask_thread():
    app.run(host='0.0.0.0', port=8080, debug=False, use_reloader=False)

def main_loop():
    global latest_frame
    last_print = time.time()
    Kp = 0.05
    Kd = 0.001
    prev_error = 0
    missed_time = None

    roi_y1, roi_y2 = 220, 240
    img_width = 320
    ref_center = 330

    stopped_due_to_red = False  # FSM 상태 변수
    
    try:
        while True:
            control = 0
            frame = cam.get_frame()
            if frame is None:
                continue

             # ---------- shb) 신호등 로직 추가함. FSM 상태 머신 로직상 얘가 제일 먼저 ----------
            if traffic_light_color == "RED":
                motor.move(0, 0)
                stopped_due_to_red = True
                latest_frame = frame  # 프레임 업데이트만
                continue  # 다음 루프로
            
            elif traffic_light_color == "GREEN" and stopped_due_to_red:
                stopped_due_to_red = False
                # → 아래 기존 경로 추종 로직 계속 실행
            
            
            elif traffic_light_color == "UNKNOWN" and stopped_due_to_red:
                # 멈춘 이후 신호등 인식 불가 시 대기 유지
                motor.move(0, 0)
                latest_frame = frame
                continue

            # === ACC(초음파) 거리 측정 및 base_speed 결정 ===
            try:
                dist_cm = ultra.get_dist()
            except:
                dist_cm = 100  # 센서 에러시 안전값
            if dist_cm < 10:
                base_speed = 0
            elif dist_cm < 20:
                base_speed = 15
            elif dist_cm < 30:
                base_speed = 30
            else:
                base_speed = 60

            # --- 1차 roi: 전체 가로 사용 ---
            roi = frame[roi_y1:roi_y2, :]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (7, 7), 0)
            _, binary = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
            indices = np.where(binary == 255)
            center_x = -1
            if len(indices[1]) > 0:
                center_x = int(np.mean(indices[1]))
            error = center_x - ref_center

           
            # --- 2차: 에러가 -50~50이면, 중앙부(가로)만 roi로 사용 ---
            if center_x != -1 and abs(error) <= 50:
                window_half = 25
                x_center = img_width // 2
                x1 = max(0, x_center - window_half)
                x2 = min(img_width, x_center + window_half)
                roi = frame[roi_y1:roi_y2, x1:x2]
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray, (7, 7), 0)
                _, binary = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
                indices = np.where(binary == 255)
                if len(indices[1]) > 0:
                    center_x = int(np.mean(indices[1])) + x1  # offset 보정!
                else:
                    center_x = -1
                error = center_x - ref_center

            if center_x == -1:
                error = 0
                left_speed = right_speed = 0
                if missed_time is None:
                    missed_time = time.time()
                if time.time() - missed_time > 1:
                    pass
                    # motor.move(0, 0)
            else:
                missed_time = None
                derivative = error - prev_error
                prev_error = error

                control = Kp * error + Kd * derivative
                if control > 5:
                    left_speed = min(base_speed + abs(int(control)), 90)
                    right_speed = max(((base_speed) - abs(int(control)) )*0.90, 0)
                elif control < -5:
                    left_speed = max(((base_speed) - abs(int(control)))*0.90, 0)
                    right_speed = min(base_speed + abs(int(control)), 90)
                else:
                    left_speed = right_speed = base_speed

                left_speed = max(0, min(left_speed, 80))
                right_speed = max(0, min(right_speed, 80))
                motor.move(left_speed, right_speed)

            

            
            
            # ===== 디버깅 표시 =====
            display_frame = frame.copy()
            if center_x != -1:
                cv2.circle(display_frame, (center_x, roi_y1+10), 8, (0,255,0), -1)
                cv2.line(display_frame, (center_x, roi_y1), (center_x, roi_y2), (0,255,0), 2)
                cv2.putText(display_frame, f"x={center_x}", (center_x+10, roi_y1+10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(display_frame, f"error={error}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(display_frame, f"dist={dist_cm:.1f}cm", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)

            latest_frame = display_frame

            # ===== LCD 출력 (0.5초에 1번) =====
            if time.time() - last_print > 0.5:
                img_disp = Image.new('RGB', (img_width, img_height), color=(0,0,0))
                draw = ImageDraw.Draw(img_disp)
                draw.text((10, 60), f"dist: {dist_cm:.1f}cm", fill=(255,255,0))
                draw.text((10, 90), f"center_x: {center_x}", fill=(255,255,255))
                draw.text((10, 120), f"error: {error}", fill=(255,255,255))
                draw.text((10, 150), f"L:{left_speed} R:{right_speed}", fill=(0,255,0))
                draw.text((10, 180), f"control: {control:.2f}", fill=(0,128,255))
                if missed_time is not None:
                    elapsed = time.time() - missed_time
                    draw.text((10, 210), f"missed: {elapsed:.1f}s", fill=(255,0,0))
                lcd.img_show(img_disp)
                last_print = time.time()

            time.sleep(0.03)
    finally:
        cam.close()
        lcd.clear()
        motor.stop()
        motor.disable_motor()
        motor.stop_motor()
        motor.clean()
        ultra.clean()  # 초음파 종료 처리

if __name__ == '__main__':
    t_flask = threading.Thread(target=flask_thread, daemon=True)
    t_flask.start()

    t_yolo = threading.Thread(target=traffic_light_detector, daemon=True)
    t_yolo.start()

    main_loop()
