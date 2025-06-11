from flask import Flask, render_template, request, jsonify
import requests
import math
import logging

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

x1, y1 = None, None
x2, y2 = None, None
x3, y3 = None, None

flag_sent_to_junghwan = False

# 각 택시의 IP
server_ip = "192.168.200.121"  # 라즈베리파이 IP
server_ip_changmin = "192.168.200.202"
server_ip_doyup = "192.168.201.68"
server_ip_junghwan = "192.168.202.131"

app = Flask(__name__)

# 최신 위치 저장용 변수
latest_position = {
    "x": None,
    "y": None,
    "robot_id": "N/A"
}

def cal_distance():
    global flag_sent_to_junghwan
    if None not in (x1, y1, point1_x, point1_y):  # 모든 값이 존재할 때만 계산
        distance = math.hypot(point1_x - x1, point1_y - y1)
        if (distance < 18) and not flag_sent_to_junghwan:
            flag_value = 2
            url_flag_junghwan = f"http://{server_ip_junghwan}:5000/flag"
            try:
                response = requests.post(url_flag_junghwan, json={"value": flag_value})
                response.raise_for_status()
                print(f"🚩 flag 2 전송 완료 (distance={distance:.2f})")
                flag_sent_to_junghwan = True  # ✔️ 이후엔 다시 안 보내짐
            except requests.RequestException as e:
                print(f"❌ flag 전송 실패: {e}")
            

        print(f" 거리: {distance:.2f}")

@app.route('/')
def crack():
    return render_template('crack.html')

@app.route('/call_taxi', methods=['POST'])
def call_taxi():
    data = request.get_json()
    start = int(data.get('start'))
    end = data.get('end')

    flag_value = 1
    if start == 1:
        url_flag_sangwoo = f"http://{server_ip}:5000/flag"
        response = requests.post(url_flag_sangwoo, json={"value": flag_value})
    elif start == 2:
        url_flag_changmin = f"http://{server_ip_changmin}:5000/flag"
        response = requests.post(url_flag_changmin, json={"value": flag_value})
    elif start == 3:
        url_flag_doyup = f"http://{server_ip_doyup}:5000/flag"
        response = requests.post(url_flag_doyup, json={"value": flag_value})
    elif start == 4:
        url_flag_junghwan = f"http://{server_ip_junghwan}:5000/flag"
        response = requests.post(url_flag_junghwan, json={"value": flag_value})
    else:
        return jsonify({'status': '잘못된 출발지 선택'}), 400

    response.raise_for_status()
    print(response.json())
    return jsonify({'status': '택시 호출됨!'})

@app.route('/taxi1_drive', methods=['POST'])
def taxi1_drive():
    flag_value = 1
    url_flag_junghwan = f"http://{server_ip_junghwan}:5000/flag"
    response = requests.post(url_flag_junghwan, json={"value": flag_value})
    print("🚗 택시1 주행")
    return jsonify({'status': '택시1 주행 명령 전송됨'})

@app.route('/taxi1_turn', methods=['POST'])
def taxi1_turn():
    flag_value = 2
    url_flag_junghwan = f"http://{server_ip_junghwan}:5000/flag"
    response = requests.post(url_flag_junghwan, json={"value": flag_value})
    print("↩️ 택시1 턴")
    return jsonify({'status': '택시1 턴 명령 전송됨'})

@app.route('/taxi1_stop', methods=['POST'])
def taxi1_stop():
    flag_value = 0
    url_flag_junghwan = f"http://{server_ip_junghwan}:5000/flag"
    response = requests.post(url_flag_junghwan, json={"value": flag_value})
    print("⛔ 택시1 멈춤")
    return jsonify({'status': '택시1 정지 명령 전송됨'})

@app.route('/taxi2_drive', methods=['POST'])
def taxi2_drive():
    flag_value = 1
    url_flag_doyup = f"http://{server_ip_doyup}:8080/flag"
    response = requests.post(url_flag_doyup, json={"value": flag_value})
    print("🚗 택시2 주행")
    return jsonify({'status': '택시2 주행 명령 전송됨'})

@app.route('/taxi2_turn', methods=['POST'])
def taxi2_turn():
    flag_value = 2
    url_flag_doyup = f"http://{server_ip_doyup}:8080/flag"
    response = requests.post(url_flag_doyup, json={"value": flag_value})
    print("↩️ 택시2 턴")
    return jsonify({'status': '택시2 턴 명령 전송됨'})

@app.route('/taxi2_stop', methods=['POST'])
def taxi2_stop():
    flag_value = 0
    url_flag_doyup = f"http://{server_ip_doyup}:8080/flag"
    response = requests.post(url_flag_doyup, json={"value": flag_value})
    print("⛔ 택시2 멈춤")
    return jsonify({'status': '택시2 정지 명령 전송됨'})

@app.route('/drop_off', methods=['POST'])
def drop_off():
    print('✅ 하차 완료')
    return jsonify({'status': '하차 완료!'})


@app.route('/pick_up', methods=['POST'])
def pick_up():
    flag_value = 1
    url_flag_junghwan = f"http://{server_ip_junghwan}:5000/flag"
    response = requests.post(url_flag_junghwan, json={"value": flag_value})
    print('🚗 승차 완료')
    return jsonify({'status': '승차 완료!'})


@app.route('/report_position1', methods=['POST'])
def report_position1():
    global x1, y1
    data = request.get_json()
    x1 = data.get("x")
    y1 = data.get("y")
   #print(f"🚘 택시1 위치 수신: x={x1}, y={y1}")
    cal_distance()
    return jsonify({"status": "received"}), 200


@app.route('/report_position2', methods=['POST'])
def report_position2():
    global x2, y2
    data = request.get_json()
    x2 = data.get("x")
    y2 = data.get("y")
    #print(f"🚘 택시2 위치 수신: x={x2}, y={y2}")

    return jsonify({"status": "received"}), 200


@app.route('/report_position3', methods=['POST'])
def report_position3():
    global x3, y3
    global point1_x, point1_y 
    data = request.get_json()
    x3 = data.get("x")
    y3 = data.get("y")

    point1_x = x3 + 90
    point1_y = y3 + 42
    #print(f"🚘 택시3 위치 수신: x={x3}, y={y3}")
    cal_distance()
    return jsonify({"status": "received"}), 200

@app.route('/get_position1', methods=['GET'])
def get_position1():
    return jsonify({"x": x1, "y": y1}), 200


@app.route('/get_position2', methods=['GET'])
def get_position2():
    return jsonify({"x": x2, "y": y2}), 200


@app.route('/get_position3', methods=['GET'])
def get_position3():
    return jsonify({"x": x3, "y": y3}), 200






if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
