import os as _os

# 환경변수 TURRET_TEST_MODE=0 이면 실제 아두이노에 연결, 그 외(기본)에는 목업 모드.
# PC 예: PowerShell에서  $env:TURRET_TEST_MODE="0"; $env:ARDUINO_PORT="COM4"; python app.py
TEST_MODE = _os.environ.get("TURRET_TEST_MODE", "1") != "0"

from flask import Flask, render_template, jsonify, request, Response, session
from flask_socketio import SocketIO
from lidar_data import get_lidar_json
import plc_client
import json
import os
import logging
from datetime import datetime
import serial
import serial.tools.list_ports
import time
import threading

app = Flask(__name__)

# 세션(관리자 로그인 유지)에 사용되는 키.
# 실제 운영 시에는 환경변수로 넘기는 걸 권장 (여기서는 기본값 제공).
app.secret_key = os.environ.get("FLASK_SECRET_KEY", "auto-turret-dev-secret-change-me")

socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ---------------------------------------------------------------------------
# 로깅 설정 (item 8)
# print() 대신 표준 logging 모듈을 사용해 콘솔 + 파일(turret.log)에 동시에 남긴다.
# 웹 UI에 보여주는 "미션 로그"는 이것과 별개로 add_log()가 계속 관리한다
# (둘의 목적이 다름: 하나는 개발자용 디버그 로그, 하나는 사용자에게 보여주는 로그).
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler("turret.log", encoding="utf-8"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("auto_turret")

# 전역 상태(current_state, data_store)와 시리얼 포트는 프로세스 내에서 공유되는
# 자원이라, 여러 요청이 동시에 들어와도 하나씩만 처리되도록 락으로 보호한다.
# 주의: gunicorn을 쓸 경우 반드시 --workers 1 로 실행할 것.
# (워커가 2개 이상이면 프로세스가 나뉘어 이 락으로도 상태 공유가 안 됨)
state_lock = threading.RLock()

ADMIN_PASSWORD = "9332"
DATA_FILE = "system_data.json"

default_data = {
    "settings": {
        str(i): {"pan": 90, "tilt": 45, "power": 30}
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
    "power": 30,
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

# 순찰(patrol) 모드가 켜져 있는지 서버가 알고 있어야, 클라이언트 없이도
# 상태를 정확히 판단할 수 있다.
patrol_state = {
    "active": False
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
            logger.info(f"[ARDUINO PORT FOUND] {port.device} - {port.description}")
            return port.device

    return None


ARDUINO_PORT = os.environ.get("ARDUINO_PORT", "")
ARDUINO_BAUD = 9600

# 아두이노 스케치(v9)의 LEFT_LIMIT_ANGLE / RIGHT_LIMIT_ANGLE / SOFT_MARGIN_DEG 와 맞출 것.
# HOME 이 끝났을 때 포의 실제 각도(왼쪽 리밋 스위치 위치)와, 아두이노가 PAN 을 받아주는 범위.
HOME_ANGLE = 53.5
PAN_MIN_ANGLE = 57     # LEFT_LIMIT_ANGLE(53.5) + SOFT_MARGIN_DEG(3) = 56.5 -> 정수로 올림
PAN_MAX_ANGLE = 144    # RIGHT_LIMIT_ANGLE(147.2) - SOFT_MARGIN_DEG(3) = 144.2 -> 정수로 내림

# 아두이노가 스스로 멈췄다는 뜻의 응답들 (웹 화면 상태도 정지로 맞춘다)
ARDUINO_STOP_ERRORS = (
    "OVERCURRENT_STOP",
    "MOVE_TIMEOUT",
    "ERROR_ENCODER_STALL",
    "ERROR_ENCODER_DIRECTION",
    "ERROR_LIMIT_LEFT",
    "ERROR_LIMIT_RIGHT",
    "ERROR_HOMING_TIMEOUT",
    "ERROR_CAL_TIMEOUT",
)
arduino = None

# 가장 최근 조준 결과 (아두이노의 AIM_OK / AIM_FAIL / 정지 오류 응답).
# /fire 가 TARGET 명령을 보낸 뒤 이 값이 바뀌길 기다렸다가 PLC 에 발사를 요청한다.
# status: None(아직 없음) | "ok" | "fail" | "error"
aim_state = {"status": None, "angle": None, "ts": 0.0, "detail": ""}

# ---------------------------------------------------------------------------
# PLC (미쓰비시 Q04UDV) 연결
#   TURRET_PLC_MODE = mock(기본) | real     TEST_MODE 가 켜져 있으면 항상 mock
#   PLC_IP (기본 192.168.0.10), PLC_PORT (기본 5010, PLC 쪽 Host Station Port 1392h; 5000~5009는 시스템 예약이라 사용 불가)
#   TURRET_PLC_MOCK_FAIL = notready | nostart | error | timeout  (mock 오류 시험용)
# ---------------------------------------------------------------------------
PLC_MODE = os.environ.get("TURRET_PLC_MODE", "mock").strip().lower()
PLC_IP = os.environ.get("PLC_IP", "192.168.0.10")
PLC_PORT = int(os.environ.get("PLC_PORT", "5010"))

AIM_CONFIRM_TIMEOUT = 12.0  # TARGET 명령 후 AIM_OK 를 기다리는 최대 시간(초)
AIM_ANGLE_TOLERANCE = 2.0   # AIM_OK 각도가 요청 각도와 이 값 이내여야 인정 (이전 MOVE 의 늦은 응답 거름)

fire_lock = threading.Lock()  # 발사 절차는 한 번에 하나만


def create_plc():
    if TEST_MODE or PLC_MODE != "real":
        return plc_client.MockPlc(fail=os.environ.get("TURRET_PLC_MOCK_FAIL", ""))
    return plc_client.MelsecPlc(PLC_IP, PLC_PORT)


plc = create_plc()
plc_status = {"connected": False, "mock": plc.is_mock, "detail": "확인 전"}

# 아두이노가 1초마다 보내는 ALIVE 하트비트를 마지막으로 받은 시각.
# 이 값이 너무 오래 갱신 안 되면 연결이 끊긴 것으로 판단한다.
arduino_last_alive = 0.0
ARDUINO_HEARTBEAT_TIMEOUT = 3.0


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
        logger.error(f"[ARDUINO ERROR] {e}")
        add_log("[ARDUINO] 연결 실패")
        return False


def handle_arduino_line(line):
    """
    아두이노가 자발적으로 보내는 줄(하트비트, 발사 완료, 안전 경고 등)을
    처리한다. 이 함수는 항상 arduino_reader_loop()의 state_lock 안에서
    호출되므로, 여기서 다시 state_lock을 잡아도(RLock) 문제없다.
    """
    global arduino_last_alive

    if line == "ALIVE":
        arduino_last_alive = time.time()
        connection_state["arduino"] = True
        return

    if line.startswith("TARGET_SET:"):
        # MOVE 명령(조이스틱/순찰)마다 매번 오는 일반 응답이라
        # 미션 로그에는 남기지 않는다 (디스크 쓰기 폭주 방지).
        return

    add_log(f"[ARDUINO RX] {line}")

    if line.startswith(ARDUINO_STOP_ERRORS):
        # 아두이노 쪽 안전장치(과전류, 엔코더 이상, 리밋 스위치 등)가 스스로 정지한 상황.
        # 웹 화면 상태도 이에 맞춰 정지로 동기화한다.
        with state_lock:
            aim_state.update(status="error", angle=None, ts=time.time(), detail=line)
            current_state["mode"] = "STOP"
            current_state["armed"] = False
            current_state["sequence"] = f"아두이노 정지: {line}"
        add_log(f"[SAFETY] 아두이노 자동 정지: {line}")
        broadcast_state()

    elif line == "ERROR_NOT_HOMED":
        with state_lock:
            current_state["sequence"] = "HOME 보정이 필요합니다"
        add_log("[SAFETY] HOME 보정 전에는 PAN 이동이 거부됩니다")
        broadcast_state()

    elif line.startswith("AIM_FAIL"):
        # 보정 횟수를 다 써도 목표 각도에 못 들어옴 -> 실제로 선 각도를 반영
        with state_lock:
            aim_state.update(status="fail", angle=None, ts=time.time(), detail=line)
            try:
                current_state["pan"] = int(round(float(line.split(":", 1)[1])))
            except Exception:
                pass
            current_state["sequence"] = "조준 실패 (목표 각도에 못 들어옴)"
        add_log("[AIM] 조준 실패 - 보정 한도 초과")
        broadcast_state()

    elif line.startswith("AIM_OK:"):
        # 아두이노가 실제로 선 각도를 알려주므로 화면 PAN 값을 그에 맞춘다
        with state_lock:
            try:
                angle = float(line.split(":", 1)[1])
                current_state["pan"] = int(round(angle))
                aim_state.update(status="ok", angle=angle, ts=time.time(), detail=line)
            except Exception:
                pass
        broadcast_state()

    elif line == "HOMED":
        with state_lock:
            current_state["pan"] = int(round(HOME_ANGLE))
            current_state["mode"] = "STANDBY"
            current_state["sequence"] = "대기 중"
        broadcast_state()


def arduino_reader_loop():
    """
    시리얼 포트에서 들어오는 줄을 읽는 유일한 담당자.
    send_arduino_command()/send_move_command()는 쓰기만 하고,
    응답 읽기는 전부 여기서 처리해서 두 곳에서 동시에 읽다가
    응답이 꼬이는 문제를 막는다.
    """
    global arduino_last_alive

    while True:
        try:
            if (
                not TEST_MODE
                and connection_state["arduino"]
                and arduino_last_alive > 0
                and (time.time() - arduino_last_alive) > ARDUINO_HEARTBEAT_TIMEOUT
            ):
                connection_state["arduino"] = False
                add_log("[ARDUINO] 하트비트 끊김 - 연결 끊긴 것으로 판단")

            if not TEST_MODE and arduino is not None and arduino.is_open:
                with state_lock:
                    if arduino.in_waiting > 0:
                        line = arduino.readline().decode(errors="ignore").strip()
                        if line:
                            handle_arduino_line(line)

        except Exception as e:
            logger.error(f"[ARDUINO READ ERROR] {e}")

        time.sleep(0.05)


def send_arduino_command(command):
    """
    발사, 정지처럼 '의미 있는 이벤트'에 사용하는 함수.
    미션 로그(add_log)에도 남기므로, 빈번하게 호출하면 로그가 금방 가득 차고
    system_data.json 디스크 쓰기가 잦아진다. 실시간 이동 명령에는
    send_move_command()를 대신 사용할 것.

    실제 하드웨어 모드에서는 쓰기만 하고 바로 반환한다 (응답은
    arduino_reader_loop()가 비동기로 받아서 로그에 남긴다). 그래서
    반환값으로 HIT/MISS를 즉시 판정하는 건 TEST_MODE에서만 유효하다.
    """
    if TEST_MODE:
        response = "AIM_OK:120"

        logger.info("[ARDUINO TEST MODE] ON")
        logger.info(f"[ARDUINO TX] {command}")
        logger.info(f"[ARDUINO RX] {response}")

        add_log("[ARDUINO TEST MODE] ON")
        add_log(f"[ARDUINO TX] {command}")
        add_log(f"[ARDUINO RX] {response}")

        return response

    if not connect_arduino():
        add_log("[ARDUINO] 연결 실패 - 명령 전송 안 됨")
        return None

    with state_lock:
        try:
            arduino.write((command + "\n").encode())
            arduino.flush()
            add_log(f"[ARDUINO TX] {command}")
            return None

        except Exception as e:
            connection_state["arduino"] = False
            add_log("[ARDUINO] 통신 오류")
            logger.error(f"[ARDUINO SEND ERROR] {e}")
            return None


def send_move_command(pan, tilt, power):
    """
    수동 조작(조이스틱)이나 순찰 모드처럼 초당 여러 번 호출될 수 있는
    '실시간 이동' 전용 함수. 미션 로그에는 남기지 않아 SD카드 쓰기 부하를
    줄인다 (필요하면 logger 파일에는 남는다).
    """
    if TEST_MODE:
        logger.debug(f"[ARDUINO MOVE TEST] PAN:{pan},TILT:{tilt},POWER:{power}")
        return "MOVE_OK"

    if not connect_arduino():
        return None

    with state_lock:
        try:
            command = f"MOVE:PAN:{pan},TILT:{tilt},POWER:{power}"
            arduino.write((command + "\n").encode())
            arduino.flush()
            return None
        except Exception as e:
            connection_state["arduino"] = False
            logger.error(f"[ARDUINO MOVE ERROR] {e}")
            return None


def send_home_command():
    """아두이노에 HOME(리미트 스위치 기준 0도 보정)을 요청한다."""
    return send_arduino_command("HOME")


def clone_default_data():
    return json.loads(json.dumps(default_data))


def save_data(data):
    with state_lock:
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

    logger.info(message)

    with state_lock:
        data_store["logs"].insert(0, log)
        data_store["logs"] = data_store["logs"][:100]
        save_data(data_store)

    try:
        socketio.emit("log_update", log)
    except Exception:
        pass


def broadcast_state():
    """현재 터렛 상태를 웹소켓으로 즉시 밀어준다 (item 5)."""
    try:
        with state_lock:
            payload = dict(current_state)
        socketio.emit("state_update", payload)
    except Exception:
        pass


def broadcast_lidar_loop():
    """
    LiDAR 데이터를 주기적으로 웹소켓 push로 보내는 백그라운드 루프.
    (이 함수를 빠뜨려서 레이더 화면 자체가 안 그려지던 문제 수정)
    실제 LiDAR가 아직 연결 안 되어 있어도 get_lidar_json()은
    '{"points":[]}' 를 반환하므로, 격자(원)는 정상적으로 계속 그려진다.
    """
    while True:
        try:
            data = json.loads(get_lidar_json())
            socketio.emit("lidar_update", data)
        except Exception as e:
            logger.error(f"[LIDAR BROADCAST ERROR] {e}")
        time.sleep(0.15)


def clamp(value, low, high):
    return max(low, min(high, value))


# ---------------------------------------------------------------------------
# LiDAR 기반 최근접 물체 탐지 / 자동 조준 (item 4)
# ---------------------------------------------------------------------------
def get_nearest_lidar_point():
    try:
        data = json.loads(get_lidar_json())
        points = data.get("points", [])
    except Exception:
        return None

    if not points:
        return None

    return min(points, key=lambda p: p["distance"])


def lidar_angle_to_pan(angle):
    """
    LiDAR가 보고하는 각도(0~359도)를 서보 pan 값(0~180)으로 변환한다.
    센서가 터렛 정면(0도)을 기준으로 장착되어 있다고 가정한 기본 매핑이며,
    실제 장착 각도/방향에 따라 보정(offset)이 필요할 수 있다.
    """
    a = angle % 360
    if a > 180:
        a = 360 - a  # 반대쪽 각도는 좌우 대칭으로 미러링
    return int(clamp(a, 0, 180))


def auto_record_result(arduino_response):
    """
    아두이노가 발사 후 명중 센서(리밋 스위치, 거리 센서 등)로 판정한 결과를
    'HIT' / 'MISS' 문자열로 포함해 응답하면, 그 값을 읽어 통계에 자동 반영한다.
    (item 6) 아두이노 펌웨어가 이 프로토콜을 아직 구현하지 않았다면 그냥
    무시되고, 화면의 수동 HIT/MISS 버튼으로 기존처럼 기록하면 된다.
    TEST_MODE 응답("AIM_OK:120")에는 HIT/MISS가 없어 자동으로는 아무 일도
    일어나지 않는다 - 실제 하드웨어 연결 후 아두이노 쪽에서
    "...,RESULT:HIT" 같은 형식으로 응답을 보내주도록 구현하면 바로 동작한다.
    """
    if not arduino_response:
        return

    resp = arduino_response.upper()

    with state_lock:
        if "HIT" in resp:
            data_store["stats"]["hit"] += 1
            save_data(data_store)
            add_log("[AUTO RESULT] HIT (자동 감지)")
        elif "MISS" in resp:
            data_store["stats"]["miss"] += 1
            save_data(data_store)
            add_log("[AUTO RESULT] MISS (자동 감지)")


@app.route("/")
def home():
    return render_template("index.html")


@app.route("/lidar")
def lidar():
    return Response(
        get_lidar_json(),
        mimetype="application/json"
    )


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
        with state_lock:
            data_store["stats"]["hit"] += 1
            save_data(data_store)
    elif result == "miss":
        with state_lock:
            data_store["stats"]["miss"] += 1
            save_data(data_store)
    else:
        return jsonify({"error": "wrong result"}), 400

    add_log(f"[RESULT] {result.upper()}")

    return jsonify({"message": "recorded"})


# ===========================================================================
# 카메라(AprilTag) 기반 TARGET 1~9 발사 흐름
# 웹사이트의 TARGET 버튼(1~9)을 누르면, 그 번호의 AprilTag를 카메라로 찾아서
# 자동으로 조준한 다음 발사한다. Pan은 카메라가 실시간으로 알려주고,
# Tilt/Power는 기존처럼 관리자 설정값(admin settings)을 그대로 사용한다.
# ===========================================================================

CAMERA_PAN_OFFSET = 90.0        # 카메라 정면 = pan 90도라고 가정 (장착 방향 다르면 조정)
CAMERA_AIM_TIMEOUT = 5.0        # 초 - 이 시간 안에 태그를 못 찾으면 발사 취소
CAMERA_AIM_SETTLE_TIME = 1.5    # 초 - 조준 명령 보낸 뒤 실제로 움직일 시간을 기다림


def camera_angle_to_pan(camera_angle):
    pan = CAMERA_PAN_OFFSET + camera_angle
    return clamp(pan, 0, 180)


# 카메라 자동조준 상태
camera_aim_state = {
    "active": False,
    "target_id": None,
}

# {target_id: {"angle": float, "updated_at": "HH:MM:SS", "ts": epoch float}}
camera_target_angles = {}


@app.route("/camera/target-angle", methods=["POST"])
def camera_target_angle():
    """camera_vision.py가 주기적으로 호출. 지금 추적 대상으로 지정된
    태그의 각도일 때만 실제로 터렛을 움직인다."""
    body = request.json or {}

    try:
        target_id = int(body["target_id"])
        angle = float(body["angle"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "target_id, angle 필요"}), 400

    now = time.time()

    with state_lock:
        camera_target_angles[target_id] = {
            "angle": angle,
            "updated_at": datetime.now().strftime("%H:%M:%S"),
            "ts": now,
        }
        should_move = (
            camera_aim_state["active"]
            and camera_aim_state["target_id"] == target_id
        )

    try:
        socketio.emit("camera_update", {"target_id": target_id, "angle": angle})
    except Exception:
        pass

    if should_move:
        pan = int(camera_angle_to_pan(angle))
        with state_lock:
            tilt = current_state["tilt"]
            power = current_state["power"]
            current_state["pan"] = pan
        send_move_command(pan, tilt, power)
        broadcast_state()

    return jsonify({"message": "ok", "moved": should_move})


@app.route("/camera/target-angles")
def get_camera_target_angles():
    return jsonify(camera_target_angles)


# ---------------------------------------------------------------------------
# 관리자 인증 (item 1)
# ---------------------------------------------------------------------------
def admin_required(func):
    from functools import wraps

    @wraps(func)
    def wrapper(*args, **kwargs):
        if not session.get("is_admin"):
            add_log("[ADMIN] 인증 없이 관리자 기능 접근 차단")
            return jsonify({"error": "관리자 로그인이 필요합니다"}), 401
        return func(*args, **kwargs)

    return wrapper


@app.route("/admin-login", methods=["POST"])
def admin_login():
    password = (request.json or {}).get("password", "")

    if password == ADMIN_PASSWORD:
        session["is_admin"] = True
        add_log("[ADMIN] 관리자 로그인 성공")
        return jsonify({"success": True})

    add_log("[ADMIN] 관리자 로그인 실패")
    return jsonify({"success": False}), 401


@app.route("/admin-logout", methods=["POST"])
def admin_logout():
    session.pop("is_admin", None)
    add_log("[ADMIN] 관리자 로그아웃")
    return jsonify({"success": True})


@app.route("/admin-status")
def admin_status():
    return jsonify({"isAdmin": bool(session.get("is_admin"))})


def wait_for_aim(pan, since_ts, timeout):
    """TARGET 명령(since_ts 이후)에 대한 아두이노의 조준 결과를 기다린다.

    돌려주는 값: (성공 여부, 실제 각도 또는 None, 실패 이유)
    요청 각도와 크게 다른 AIM_OK 는 이전 MOVE 명령의 늦은 응답일 수 있어 무시한다.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        with state_lock:
            st = dict(aim_state)
        if st["ts"] >= since_ts:
            if st["status"] == "ok" and abs(st["angle"] - pan) <= AIM_ANGLE_TOLERANCE:
                return True, st["angle"], ""
            if st["status"] == "fail":
                return False, None, "조준 실패 (목표 각도에 못 들어옴)"
            if st["status"] == "error":
                return False, None, f"아두이노 정지: {st['detail']}"
        time.sleep(0.05)
    return False, None, "조준 완료(AIM_OK) 신호를 받지 못함"


def _fire_abort(num, message, http_status):
    with state_lock:
        current_state["mode"] = "STANDBY"
        current_state["sequence"] = "대기 중"
    broadcast_state()
    add_log(f"[FIRE] TARGET {num:02d} 취소 - {message}")
    return jsonify({"error": f"TARGET {num:02d} 발사 취소: {message}"}), http_status


@app.route("/fire/<int:num>")
def fire(num):
    """
    TARGET 버튼(1~9) 클릭 시 실행되는 흐름:
      1. 해당 번호의 AprilTag를 카메라 자동조준 대상으로 지정
      2. 카메라가 그 태그를 인식해서 각도를 보내올 때까지 대기(최대 5초)
      3. 인식되면 그 각도로 이동시키고, 움직일 시간 잠깐 대기
      4. 아두이노에 TARGET 명령을 보내고 AIM_OK(조준 완료)를 기다림
      5. PLC 에 발사 요청(M3 조준 완료 + M4 발사 요청) -> PLC 가 실린더/에어밸브 시퀀스 수행
         -> M5(발사 완료)를 받으면 끝. (Tilt/Power는 관리자 설정값 그대로 사용)
      6. 어느 단계든 실패하면 발사하지 않고 취소
    """
    if num < 1 or num > 9:
        return "wrong target", 400

    if not fire_lock.acquire(blocking=False):
        return jsonify({"error": "다른 발사 절차가 진행 중입니다"}), 409

    try:
        return _do_fire(num)
    finally:
        fire_lock.release()


def _do_fire(num):
    with state_lock:
        s = data_store["settings"][str(num)]
        tilt = s["tilt"]
        power = s["power"]

        camera_aim_state["active"] = True
        camera_aim_state["target_id"] = num
        current_state["target"] = num
        current_state["mode"] = "CAMERA_AIMING"
        current_state["sequence"] = f"{num}번 태그 탐색 중"

    broadcast_state()
    add_log(f"[CAMERA AIM] TARGET {num:02d} - AprilTag 탐색 시작")

    # 이 시점 이후에 새로 들어오는 각도만 "이번 조준 시도의 결과"로 인정.
    # (이전에 저장되어 있던 오래된 각도값을 잘못 재사용하는 것 방지)
    request_start_ts = time.time()
    deadline = request_start_ts + CAMERA_AIM_TIMEOUT
    found = False

    while time.time() < deadline:
        with state_lock:
            info = camera_target_angles.get(num)

        if info is not None and info["ts"] > request_start_ts:
            found = True
            break

        time.sleep(0.1)

    with state_lock:
        camera_aim_state["active"] = False

    if not found:
        with state_lock:
            current_state["mode"] = "STANDBY"
            current_state["sequence"] = "대기 중"
        broadcast_state()
        add_log(f"[CAMERA AIM] TARGET {num:02d} - 태그를 찾지 못함, 발사 취소")
        return jsonify({"error": f"TARGET {num:02d} AprilTag를 찾지 못했습니다"}), 404

    # 조준 명령이 나간 뒤 실제로 모터가 움직일 시간을 기다림
    time.sleep(CAMERA_AIM_SETTLE_TIME)

    # --- 아두이노 조준 확인 -------------------------------------------------
    with state_lock:
        pan = current_state["pan"]
        current_state["mode"] = "AIMING"
        current_state["sequence"] = "조준 확인 중"
        aim_state.update(status=None, angle=None, ts=0.0, detail="")

    broadcast_state()

    send_ts = time.time()
    response = send_arduino_command(
        f"TARGET:{num},PAN:{pan},TILT:{tilt},POWER:{power}"
    )

    aimed_angle = pan
    if TEST_MODE:
        pass  # 목업 아두이노는 항상 즉시 AIM_OK 를 돌려준다
    else:
        if not connection_state["arduino"]:
            return _fire_abort(num, "아두이노와 통신할 수 없음", 503)
        ok, angle, why = wait_for_aim(pan, send_ts, AIM_CONFIRM_TIMEOUT)
        if not ok:
            return _fire_abort(num, why, 500)
        aimed_angle = angle

    # --- PLC 발사 -----------------------------------------------------------
    with state_lock:
        current_state["mode"] = "FIRING"
        current_state["sequence"] = "PLC 발사 중"

    broadcast_state()

    # 블로킹 구간이라 state_lock 을 잡지 않는다 (아두이노 읽기 스레드가 막히지 않게)
    result = plc_client.run_fire_sequence(
        plc, num, aimed_angle, tilt, power, log=add_log
    )

    if not result.ok:
        add_log(f"[PLC] 발사 실패 ({result.stage}): {result.message}")
        http_status = {"comm": 503, "check": 409, "ready": 409}.get(result.stage, 500)
        return _fire_abort(num, result.message, http_status)

    with state_lock:
        data_store["stats"]["totalFire"] += 1
        save_data(data_store)

    add_log(f"[FIRE] TARGET {num:02d} (AprilTag 자동조준 + PLC) 발사 완료")
    auto_record_result(response)

    with state_lock:
        current_state["mode"] = "STANDBY"
        current_state["sequence"] = "대기 중"

    broadcast_state()

    return f"TARGET {num:02d} 발사 완료 (AprilTag 조준)"


def plc_status_loop():
    """1초마다 PLC 상태를 읽어 연결 표시(connection_state)를 갱신한다."""
    while True:
        try:
            st = plc_client.read_status(plc)
            connection_state["plc"] = True
            # M0(시스템 준비)가 꺼져 있으면 비상정지가 눌린 것으로 본다
            connection_state["emergencyStop"] = bool(st["system_ready"])
            plc_status.update(connected=True, detail="정상", **{"status": st})
        except plc_client.PlcError as e:
            connection_state["plc"] = False
            plc_status.update(connected=False, detail=str(e), status=None)
        except Exception as e:  # 읽기 스레드는 어떤 경우에도 죽지 않게
            logger.error(f"[PLC STATUS ERROR] {e}")
        time.sleep(1.0)


@app.route("/plc/status")
def plc_status_route():
    return jsonify(plc_status)


@app.route("/manual", methods=["POST"])
def manual_control():
    body = request.json or {}

    # 아두이노가 받아주는 범위(PAN_MIN_ANGLE~PAN_MAX_ANGLE)로 미리 제한해서 화면 값과 실제가 어긋나지 않게 함
    pan = int(clamp(int(body.get("pan", current_state["pan"])), PAN_MIN_ANGLE, PAN_MAX_ANGLE))
    tilt = int(clamp(int(body.get("tilt", current_state["tilt"])), 0, 180))
    power = int(clamp(int(body.get("power", current_state["power"])), 10, 3000))

    with state_lock:
        current_state["pan"] = pan
        current_state["tilt"] = tilt
        current_state["power"] = power
        current_state["mode"] = "MANUAL"

    # 이전에는 여기서 상태만 바뀌고 실제 아두이노에는 아무 것도 전송되지
    # 않아, 조이스틱/순찰 모드를 움직여도 물리적으로는 안 움직였다.
    send_move_command(pan, tilt, power)
    broadcast_state()

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

    with state_lock:
        pan = current_state["pan"]
        tilt = current_state["tilt"]
        power = current_state["power"]

        data_store["stats"]["totalFire"] += 1
        save_data(data_store)

        current_state["mode"] = "MANUAL_FIRE"
        current_state["sequence"] = "수동 발사"

    broadcast_state()

    # 기존에는 로그만 남기고 실제 시리얼 명령이 전송되지 않았음
    response = send_arduino_command(f"MANUAL_FIRE:PAN:{pan},TILT:{tilt},POWER:{power}")

    add_log("[MANUAL FIRE] 수동 발사 완료")
    auto_record_result(response)

    with state_lock:
        current_state["mode"] = "MANUAL"
        current_state["sequence"] = "대기 중"

    broadcast_state()

    return "수동 발사 완료"


@app.route("/arm", methods=["POST"])
def arm_system():
    with state_lock:
        current_state["armed"] = bool((request.json or {}).get("armed", False))
        armed = current_state["armed"]

    add_log(f"[ARM] {'ON' if armed else 'OFF'}")
    broadcast_state()

    return jsonify({"armed": armed})


@app.route("/patrol", methods=["POST"])
def patrol():
    """
    이전에는 상태값만 'PATROL'로 바꿀 뿐, 끌 때 서버에 알리는 방법이
    없어서 서버 쪽 상태가 영원히 PATROL로 남는 문제가 있었다.
    이제 on/off를 명시적으로 받고, 끌 때는 중앙(90도)으로 복귀시킨다.
    """
    body = request.json or {}
    on = bool(body.get("on", True))

    with state_lock:
        patrol_state["active"] = on
        current_state["mode"] = "PATROL" if on else "STANDBY"
        current_state["sequence"] = "자동 순찰" if on else "대기 중"

    add_log(f"[PATROL] 자동 순찰 모드 {'실행' if on else '종료'}")

    if not on:
        with state_lock:
            current_state["pan"] = 90
            current_state["tilt"] = 45
        send_move_command(90, 45, current_state["power"])

    broadcast_state()

    return jsonify({"mode": current_state["mode"], "active": on})


@app.route("/auto-aim", methods=["POST"])
def auto_aim():
    """
    LiDAR로 감지된 가장 가까운 물체 방향으로 자동 조준한다 (item 4).
    ARM이 켜져 있으면 조준 후 바로 발사까지 수행한다.
    """
    point = get_nearest_lidar_point()

    if point is None:
        return jsonify({"message": "감지된 물체 없음"}), 404

    pan = lidar_angle_to_pan(point["angle"])
    tilt = 45

    with state_lock:
        power = current_state["power"]
        current_state["pan"] = pan
        current_state["tilt"] = tilt
        current_state["mode"] = "AUTO_AIM"
        current_state["sequence"] = "자동 조준"

    broadcast_state()
    send_move_command(pan, tilt, power)
    add_log(f"[AUTO AIM] 최근접 물체 각도 {point['angle']}도, 거리 {point['distance']}cm -> PAN {pan}")

    fired = False

    if current_state.get("armed"):
        response = send_arduino_command(f"TARGET:AUTO,PAN:{pan},TILT:{tilt},POWER:{power}")
        add_log("[AUTO AIM] ARM 상태 - 자동 발사 실행")

        with state_lock:
            data_store["stats"]["totalFire"] += 1
            save_data(data_store)

        auto_record_result(response)
        fired = True

    with state_lock:
        current_state["mode"] = "STANDBY"
        current_state["sequence"] = "대기 중"

    broadcast_state()

    return jsonify({
        "message": "자동 조준 완료",
        "pan": pan,
        "tilt": tilt,
        "distance": point["distance"],
        "fired": fired
    })


@app.route("/stop", methods=["POST", "GET"])
def stop():
    with state_lock:
        current_state["mode"] = "STOP"
        current_state["armed"] = False
        current_state["sequence"] = "긴급 정지"
        patrol_state["active"] = False

    # 이전에는 상태만 바뀌고 실제 하드웨어에는 정지 신호가 전달되지 않았다.
    send_arduino_command("STOP")
    add_log("[STOP] 긴급 정지")
    broadcast_state()

    return "긴급 정지 완료"


@app.route("/home-calibrate", methods=["POST"])
def home_calibrate():
    """
    아두이노에 HOME 명령을 보내 리미트 스위치 기준으로 0도를 다시 잡는다.
    (엔코더 드리프트나 재부팅 후 위치 불일치를 바로잡기 위한 캘리브레이션)
    """
    with state_lock:
        current_state["mode"] = "HOMING"
        current_state["sequence"] = "홈 위치 보정 중"

    add_log("[HOME] 홈 위치 보정 시작")
    send_home_command()
    broadcast_state()

    return jsonify({"message": "홈 위치 보정을 시작했습니다"})


@app.route("/settings")
def get_settings():
    return jsonify(data_store["settings"])


@app.route("/settings/<int:num>", methods=["POST"])
@admin_required
def save_settings(num):
    if num < 1 or num > 9:
        return jsonify({"error": "wrong target"}), 400

    body = request.json or {}

    # 범위를 벗어난 값이 그대로 저장되던 문제 수정 (item 7)
    pan = int(clamp(int(body["pan"]), 0, 180))
    tilt = int(clamp(int(body["tilt"]), 0, 180))
    power = int(clamp(int(body["power"]), 10, 3000))

    with state_lock:
        data_store["settings"][str(num)] = {
            "pan": pan,
            "tilt": tilt,
            "power": power
        }
        save_data(data_store)

    add_log(f"[SAVE] TARGET {num:02d} 설정 저장 (PAN:{pan}, TILT:{tilt}, POWER:{power})")

    return jsonify({"message": f"TARGET {num:02d} 저장 완료"})


@app.route("/reset-data", methods=["POST"])
@admin_required
def reset_data():
    global data_store
    with state_lock:
        data_store = clone_default_data()
        save_data(data_store)
    add_log("[SYSTEM] 데이터 초기화")
    return jsonify({"message": "데이터 초기화 완료"})


if __name__ == "__main__":
    socketio.start_background_task(broadcast_lidar_loop)
    socketio.start_background_task(arduino_reader_loop)
    socketio.start_background_task(plc_status_loop)
    add_log(
        "[PLC] MOCK 모드 (PLC 없이 가짜 응답)" if plc.is_mock
        else f"[PLC] 실제 PLC 연결 시도: {PLC_IP}:{PLC_PORT}"
    )
    # 이 프로젝트는 라즈베리파이 한 대에서만 개인적으로 운영하는 용도라
    # allow_unsafe_werkzeug=True로 프로덕션 경고를 해제한다.
    socketio.run(app, host="0.0.0.0", port=5000, allow_unsafe_werkzeug=True)
