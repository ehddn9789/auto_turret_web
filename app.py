TEST_MODE = True

from flask import Flask, render_template, jsonify, request
from lidar_data import get_lidar_points
import json
import os
from datetime import datetime
import serial
import serial.tools.list_ports
import time

app = Flask(__name__)

ADMIN_PASSWORD = "9332"
DATA_FILE = "system_data.json"

default_data = {
    "settings": {
        str(i): {"pan": 90, "tilt": 45, "power": 300}
        for i in range(1, 10)
    },
    "stats": {
        "totalFire": 0,
        "hit": 0,
        "miss": 0
    },
    "logs": []
}

current_state = {
    "pan": 90,
    "tilt": 45,
    "power": 300,
    "target": None,
    "mode": "STANDBY",
    "armed": False,
    "sequence": "대기 중"
}

connection_state = {
    "lidar": True,
    "arduino": False,
    "plc": False,
    "airValve": False,
    "emergencyStop": True
}

def find_arduino_port():
    ports = serial.tools.list_ports.comports()

    for port in ports:
        desc = port.description.lower()
        hwid = port.hwid.lower()

        if (
            "arduino" in desc
            or "ch340" in desc
            or "ch341" in desc
            or "usb serial" in desc
            or "usb-serial" in desc
            or "cp210" in desc
            or "ttyacm" in port.device.lower()
            or "ttyusb" in port.device.lower()
        ):
            print(f"[ARDUINO PORT FOUND] {port.device} - {port.description}")
            return port.device

    return None

ARDUINO_PORT = os.environ.get("ARDUINO_PORT", "")
ARDUINO_BAUD = 9600
arduino = None


def connect_arduino():
    global arduino

    try:
        if arduino is not None and arduino.is_open:
            return True

        port_name = ARDUINO_PORT

        if not port_name:
            port_name = find_arduino_port()

        if not port_name:
            connection_state["arduino"] = False
            add_log("[ARDUINO] 포트 자동 검색 실패")
            return False

        arduino = serial.Serial(port_name, ARDUINO_BAUD, timeout=2)
        time.sleep(2)

        connection_state["arduino"] = True
        add_log(f"[ARDUINO] 연결 성공: {port_name}")

        return True

    except Exception as e:
        connection_state["arduino"] = False
        print("[ARDUINO ERROR]", e)
        add_log("[ARDUINO] 연결 실패")
        return False


def send_arduino_command(command):
    if TEST_MODE:
        response = "AIM_OK:120"

        print(f"[ARDUINO TEST MODE] ON")
        print(f"[ARDUINO TX] {command}")
        print(f"[ARDUINO RX] {response}")

        add_log("[ARDUINO TEST MODE] ON")
        add_log(f"[ARDUINO TX] {command}")
        add_log(f"[ARDUINO RX] {response}")

        return response

    if not connect_arduino():
        add_log("[ARDUINO] 연결 실패 - 명령 전송 안 됨")
        return None

    try:
        arduino.write((command + "\n").encode())
        arduino.flush()

        response = arduino.readline().decode(errors="ignore").strip()

        add_log(f"[ARDUINO TX] {command}")
        if response:
            add_log(f"[ARDUINO RX] {response}")

        return response

    except Exception as e:
        connection_state["arduino"] = False
        add_log("[ARDUINO] 통신 오류")
        print("[ARDUINO SEND ERROR]", e)
        return None

def clone_default_data():
    return json.loads(json.dumps(default_data))


def save_data(data):
    with open(DATA_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def load_data():
    if not os.path.exists(DATA_FILE):
        data = clone_default_data()
        save_data(data)
        return data

    try:
        with open(DATA_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        data = clone_default_data()
        save_data(data)
        return data

    if "settings" not in data:
        data["settings"] = clone_default_data()["settings"]
    if "stats" not in data:
        data["stats"] = clone_default_data()["stats"]
    if "logs" not in data:
        data["logs"] = []

    return data


data_store = load_data()


def add_log(message):
    log = {
        "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "message": message
    }

    data_store["logs"].insert(0, log)
    data_store["logs"] = data_store["logs"][:100]
    save_data(data_store)


@app.route("/")
def home():
    return render_template("index.html")


@app.route("/lidar")
def lidar():
    return jsonify({"points": get_lidar_points()})


@app.route("/state")
def state():
    return jsonify(current_state)


@app.route("/connections")
def connections():
    return jsonify(connection_state)


@app.route("/checklist")
def checklist():
    return jsonify({
        "LiDAR Sensor": connection_state["lidar"],
        "Arduino Controller": connection_state["arduino"],
        "PLC / HMI Link": connection_state["plc"],
        "Air Valve": connection_state["airValve"],
        "Emergency Stop": connection_state["emergencyStop"],
        "Target Settings": True
    })


@app.route("/stats")
def get_stats():
    stats = data_store["stats"]
    total = stats["totalFire"]
    hit = stats["hit"]
    hit_rate = round((hit / total) * 100, 1) if total > 0 else 0

    return jsonify({
        "totalFire": total,
        "hit": hit,
        "miss": stats["miss"],
        "hitRate": hit_rate
    })


@app.route("/logs")
def get_logs():
    return jsonify(data_store["logs"])


@app.route("/record/<result>", methods=["POST"])
def record_result(result):
    if result == "hit":
        data_store["stats"]["hit"] += 1
    elif result == "miss":
        data_store["stats"]["miss"] += 1
    else:
        return jsonify({"error": "wrong result"}), 400

    add_log(f"[RESULT] {result.upper()}")
    save_data(data_store)

    return jsonify({"message": "recorded"})


@app.route("/admin-login", methods=["POST"])
def admin_login():
    password = request.json.get("password", "")

    if password == ADMIN_PASSWORD:
        add_log("[ADMIN] 관리자 로그인 성공")
        return jsonify({"success": True})

    add_log("[ADMIN] 관리자 로그인 실패")
    return jsonify({"success": False}), 401


@app.route("/fire/<int:num>")
def fire(num):
    if num < 1 or num > 9:
        return "wrong target", 400

    s = data_store["settings"][str(num)]

    current_state["pan"] = s["pan"]
    current_state["tilt"] = s["tilt"]
    current_state["power"] = s["power"]
    current_state["target"] = num
    current_state["mode"] = "FIRING"
    current_state["sequence"] = "발사"

    data_store["stats"]["totalFire"] += 1
    save_data(data_store)

    print(f"[AUTO FIRE] TARGET {num}")
    print(f"PAN={s['pan']}, TILT={s['tilt']}, POWER={s['power']}")
    send_arduino_command(f"PAN:{s['pan']}")

    # 실제 아두이노 연결 시 이 위치에 Serial 전송 코드를 넣으면 됨
    # 예: TARGET:1,PAN:90,TILT:45,POWER:300

    add_log(f"[FIRE] TARGET {num:02d} 발사 완료")

    current_state["mode"] = "STANDBY"
    current_state["sequence"] = "대기 중"

    return f"TARGET {num:02d} 발사 완료"


@app.route("/manual", methods=["POST"])
def manual_control():
    body = request.json

    pan = max(0, min(180, int(body.get("pan", current_state["pan"]))))
    tilt = max(0, min(180, int(body.get("tilt", current_state["tilt"]))))
    power = max(50, min(3000, int(body.get("power", current_state["power"]))))

    current_state["pan"] = pan
    current_state["tilt"] = tilt
    current_state["power"] = power
    current_state["mode"] = "MANUAL"

    return jsonify({
        "message": "manual updated",
        "pan": pan,
        "tilt": tilt,
        "power": power
    })


@app.route("/manual-fire", methods=["POST"])
def manual_fire():
    if not current_state["armed"]:
        return "ARMED OFF - 발사 차단", 403

    data_store["stats"]["totalFire"] += 1
    save_data(data_store)

    current_state["mode"] = "MANUAL_FIRE"
    current_state["sequence"] = "수동 발사"

    add_log("[MANUAL FIRE] 수동 발사 완료")

    current_state["mode"] = "MANUAL"
    current_state["sequence"] = "대기 중"

    return "수동 발사 완료"


@app.route("/arm", methods=["POST"])
def arm_system():
    current_state["armed"] = bool(request.json.get("armed", False))
    add_log(f"[ARM] {'ON' if current_state['armed'] else 'OFF'}")

    return jsonify({"armed": current_state["armed"]})


@app.route("/patrol")
def patrol():
    current_state["mode"] = "PATROL"
    add_log("[PATROL] 자동 순찰 모드 실행")
    return "자동 순찰 모드 실행"


@app.route("/stop")
def stop():
    current_state["mode"] = "STOP"
    current_state["armed"] = False
    current_state["sequence"] = "긴급 정지"
    add_log("[STOP] 긴급 정지")
    return "긴급 정지 완료"


@app.route("/settings")
def get_settings():
    return jsonify(data_store["settings"])


@app.route("/settings/<int:num>", methods=["POST"])
def save_settings(num):
    if num < 1 or num > 9:
        return jsonify({"error": "wrong target"}), 400

    body = request.json

    data_store["settings"][str(num)] = {
        "pan": int(body["pan"]),
        "tilt": int(body["tilt"]),
        "power": int(body["power"])
    }

    save_data(data_store)
    add_log(f"[SAVE] TARGET {num:02d} 설정 저장")

    return jsonify({"message": f"TARGET {num:02d} 저장 완료"})


@app.route("/reset-data", methods=["POST"])
def reset_data():
    global data_store
    data_store = clone_default_data()
    save_data(data_store)
    add_log("[SYSTEM] 데이터 초기화")
    return jsonify({"message": "데이터 초기화 완료"})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
