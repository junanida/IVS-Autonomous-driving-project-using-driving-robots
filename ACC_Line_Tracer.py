

import threading
from flask import Flask, Response, request, jsonify
import cv2
from pinkylib import Camera, Motor, Ultrasonic
from pinky_lcd import LCD
from PIL import Image, ImageDraw
import numpy as np
import time

app = Flask(__name__)

cam = Camera()
cam.start()
motor = Motor()
lcd = LCD()
ultra = Ultrasonic()
motor.enable_motor()
motor.start_motor()

img_width, img_height = 320, 240
latest_frame = None

# ===== 전역 상태 =====
flag = 0  # 0: 정지, 1: 주행, 2: 탱크턴, 3: 탱크턴 후 1초 주행
start_time_flag3 = None

@app.route('/flag', methods=['GET'])
def get_flag():
    return jsonify({"flag": flag})

@app.route('/flag', methods=['POST'])
def set_flag():
    global flag
    try:
        value = int(request.json.get("value"))
        if value not in [0, 1, 2, 3]:
            raise ValueError("Only 0, 1, 2, 3 allowed")
        flag = value
        return jsonify({"message": f"Flag set to {flag}"})
    except Exception as e:
        return jsonify({"error": str(e)}), 400

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
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def flask_thread():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def tank_turn_once(speed=30, duration=0.265):
    motor.move(speed, -speed)
    time.sleep(duration)
    motor.stop()

def main_loop():
    global latest_frame, flag, start_time_flag3
    last_print = time.time()
    Kp = 0.03
    Kd = 0.001
    prev_error = 0
    missed_time = None

    roi_y1, roi_y2 = 220, 240
    ref_center = 310

    try:
        while True:
            # --- 탱크턴 ---
            # if flag == 2:
            #     motor.stop()
            #     time.sleep(1)
            #     tank_turn_once(speed=30, duration=0.3)
            #     flag = 3
            #     start_time_flag3 = time.time()
            #     print("탱크턴 완료 → flag=3")
            #     continue

            if flag == 2:
                motor.stop()
                time.sleep(1)
                tank_turn_once(speed=30, duration=0.3)
                motor.stop()
                flag = 0  # 바로 정지 상태로
                print("탱크턴 완료 → flag=0 (정지)")
                continue


            # --- flag=3: 1초 주행 후 정지 ---
            if flag == 3 and start_time_flag3 is not None:
                if time.time() - start_time_flag3 > 1.0:
                    motor.stop()
                    flag = 0
                    start_time_flag3 = None
                    print("1초 주행 완료 → flag=0")
                    continue

            # --- flag=1 또는 3 이외에는 정지 ---
            if flag not in [1, 3]:
                motor.stop()
                time.sleep(0.05)
                continue

            frame = cam.get_frame()
            if frame is None:
                continue

            # 거리 기반 속도 조절
            try:
                dist_cm = ultra.get_dist()
            except:
                dist_cm = 100
            if dist_cm < 10:
                base_speed = 0
            elif dist_cm < 20:
                base_speed = 15
            elif dist_cm < 30:
                base_speed = 30
            else:
                base_speed = 45

            # 사이드 차선 인식
            left_roi = frame[roi_y1:roi_y2, 0:80]
            left_x = detect_line_center(left_roi)
            right_roi = frame[roi_y1:roi_y2, 240:320]
            right_x = detect_line_center(right_roi)
            # 중앙선
            roi = frame[roi_y1:roi_y2, :]
            center_x_central = detect_line_center(roi)

            # 중심선 선택
            if left_x != -1 and right_x != -1:
                center_x = (left_x + right_x) // 2
                tracking_type = "side_both"
            elif left_x != -1:
                center_x = left_x
                tracking_type = "side_left"
            elif right_x != -1:
                center_x = right_x
                tracking_type = "side_right"
            else:
                center_x = center_x_central
                tracking_type = "center"

            error = center_x - ref_center if center_x != -1 else 0

            # PID 제어
            if center_x == -1:
                error = 0
                left_speed = right_speed = 0
                if missed_time is None:
                    missed_time = time.time()
            else:
                missed_time = None
                derivative = error - prev_error
                prev_error = error
                control = Kp * error + Kd * derivative
                if control > 5:
                    left_speed = min(base_speed + abs(int(control)), 90)
                    right_speed = max((base_speed - abs(int(control))) * 0.90, 0)
                elif control < -5:
                    left_speed = max((base_speed - abs(int(control))) * 0.90, 0)
                    right_speed = min(base_speed + abs(int(control)), 90)
                else:
                    left_speed = right_speed = base_speed

                left_speed = max(0, min(left_speed, 80))
                right_speed = max(0, min(right_speed, 80))
                motor.move(left_speed, right_speed)

            # 디버그 프레임 시각화
            display_frame = frame.copy()
            for x, color in [(left_x, (255, 0, 0)), (right_x, (0, 0, 255)), (center_x_central, (128, 128, 128)), (center_x, (0, 255, 0))]:
                if x != -1:
                    cv2.circle(display_frame, (x, roi_y1 + 10), 6, color, -1)
            cv2.putText(display_frame, f"error={error}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(display_frame, f"dist={dist_cm:.1f}cm", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(display_frame, f"tracking={tracking_type}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            latest_frame = display_frame

            # LCD 출력
            if time.time() - last_print > 0.5:
                img_disp = Image.new('RGB', (img_width, img_height), color=(0, 0, 0))
                draw = ImageDraw.Draw(img_disp)
                draw.text((10, 60), f"dist: {dist_cm:.1f}cm", fill=(255, 255, 0))
                draw.text((10, 90), f"center_x: {center_x}", fill=(255, 255, 255))
                draw.text((10, 120), f"error: {error}", fill=(255, 255, 255))
                draw.text((10, 150), f"L:{left_speed} R:{right_speed}", fill=(0, 255, 0))
                draw.text((10, 180), f"control: {control:.2f}", fill=(0, 128, 255))
                draw.text((10, 210), f"tracking: {tracking_type}", fill=(255, 128, 0))
                if missed_time is not None:
                    elapsed = time.time() - missed_time
                    draw.text((10, 240), f"missed: {elapsed:.1f}s", fill=(255, 0, 0))
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
        ultra.clean()

def detect_line_center(roi):
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    _, binary = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
    indices = np.where(binary == 255)
    if len(indices[1]) > 0:
        return int(np.mean(indices[1]))
    else:
        return -1

if __name__ == '__main__':
    t_flask = threading.Thread(target=flask_thread, daemon=True)
    t_flask.start()
    main_loop()
