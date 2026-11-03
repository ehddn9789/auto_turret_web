"""app.py 의 /fire 전체 흐름 시험 (serial / flask_socketio 는 없으면 가짜로 대체). 실행: python3 tests/test_app_fire.py"""
import os, sys, types, threading, time, tempfile
root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, root)
os.chdir(tempfile.mkdtemp())          # turret.log / system_data.json 이 저장소를 더럽히지 않게
os.environ["TURRET_TEST_MODE"] = os.environ.get("T_MODE", "1")
os.environ.setdefault("TURRET_PLC_MODE", "mock")
try:
    import serial, serial.tools.list_ports  # noqa
except ImportError:
    ser = types.ModuleType("serial"); ser.tools = types.ModuleType("serial.tools")
    ser.tools.list_ports = types.ModuleType("serial.tools.list_ports"); ser.tools.list_ports.comports = lambda: []
    ser.Serial = object
    sys.modules.update({"serial": ser, "serial.tools": ser.tools, "serial.tools.list_ports": ser.tools.list_ports})
try:
    import flask_socketio  # noqa
except ImportError:
    fs = types.ModuleType("flask_socketio")
    class SocketIO:
        def __init__(self, *a, **k): pass
        def emit(self, *a, **k): pass
        def start_background_task(self, f, *a, **k): threading.Thread(target=f, args=a, daemon=True).start()
        def run(self, *a, **k): pass
    fs.SocketIO = SocketIO; sys.modules["flask_socketio"] = fs
sys.modules.setdefault("lidar_data", types.SimpleNamespace(get_lidar_json=lambda: "{}"))
import app as A
A.CAMERA_AIM_SETTLE_TIME = 0.1; A.CAMERA_AIM_TIMEOUT = 1.0
c = A.app.test_client()
fails = 0
def check(n, cond):
    global fails; print(("PASS " if cond else "FAIL ") + n); fails += (not cond)

def camera_feeder(num, angle=10.0):
    def go():
        time.sleep(0.2)
        c.post("/camera/target-angle", json={"target_id": num, "angle": angle})
    threading.Thread(target=go, daemon=True).start()

def set_plc(fail, step=0.1):
    A.plc = A.plc_client.MockPlc(fail=fail, step_seconds=step)

before = A.data_store["stats"]["totalFire"]
set_plc(""); camera_feeder(2)
r = c.get("/fire/2"); check("normal: 200", r.status_code == 200 and "발사 완료" in r.get_data(as_text=True))
check("normal: totalFire +1", A.data_store["stats"]["totalFire"] == before + 1)
check("normal: back to STANDBY", A.current_state["mode"] == "STANDBY")

for fail, code in (("error", 500), ("timeout", 500), ("nostart", 500), ("notready", 409)):
    set_plc(fail); A.plc_client.TIMEOUTS.update(ready=0.5, start=0.5, done=1.0)
    n0 = A.data_store["stats"]["totalFire"]; camera_feeder(3)
    r = c.get("/fire/3"); body = r.get_json()
    check(f"{fail}: status {code}", r.status_code == code)
    check(f"{fail}: not counted, STANDBY", A.data_store["stats"]["totalFire"] == n0 and A.current_state["mode"] == "STANDBY")
    check(f"{fail}: error message present", bool(body and body.get("error")))

r = c.get("/fire/4"); check("no tag -> 404", r.status_code == 404)
r = c.get("/fire/12"); check("bad number -> 400", r.status_code == 400)
check("/plc/status", c.get("/plc/status").get_json().get("mock") is True)
check("fire_lock released", A.fire_lock.acquire(blocking=False) and (A.fire_lock.release() or True))

# 실제 하드웨어 경로 (TEST_MODE 꺼짐) 시뮬레이션: 아두이노 응답을 직접 주입
if not A.TEST_MODE:
    pass
A.TEST_MODE = False
A.connection_state["arduino"] = True
sent = []
A.send_arduino_command = lambda cmd: sent.append(cmd)
A.send_move_command = lambda *a, **k: None
def run_real(reply, pan_expect=None):
    set_plc(""); A.plc_client.TIMEOUTS.update(ready=1.0, start=1.0, done=3.0)
    A.AIM_CONFIRM_TIMEOUT = 1.0
    sent.clear()
    camera_feeder(5, 20.0)
    def arduino_reply():
        while not sent: time.sleep(0.01)
        time.sleep(0.2)
        if reply: A.handle_arduino_line(reply(A.current_state["pan"]))
    threading.Thread(target=arduino_reply, daemon=True).start()
    return c.get("/fire/5")
n0 = A.data_store["stats"]["totalFire"]
r = run_real(lambda pan: f"AIM_OK:{pan}.4"); check("real: AIM_OK -> fire ok", r.status_code == 200 and A.data_store["stats"]["totalFire"] == n0 + 1)
check("real: D1 = pan*10 from AIM_OK angle", A.plc.read_words("D1", 1)[0] % 10 == 4)
n0 = A.data_store["stats"]["totalFire"]
r = run_real(lambda pan: "AIM_FAIL:70"); check("real: AIM_FAIL -> 500, PLC untouched", r.status_code == 500 and A.plc.read_bits("M4", 1) == [0] and A.plc.read_words("D0", 1) == [0] and A.data_store["stats"]["totalFire"] == n0)
r = run_real(lambda pan: "MOVE_TIMEOUT"); check("real: MOVE_TIMEOUT -> 500", r.status_code == 500 and "아두이노 정지" in r.get_json()["error"])
r = run_real(None); check("real: no AIM_OK -> 500 timeout", r.status_code == 500 and "AIM_OK" in r.get_json()["error"])
r = run_real(lambda pan: f"AIM_OK:{pan + 30}"); check("real: stale AIM_OK (wrong angle) ignored -> timeout", r.status_code == 500)
A.connection_state["arduino"] = False
camera_feeder(6); r = c.get("/fire/6"); check("real: arduino down -> 503", r.status_code == 503)

print("FAILED:", fails); sys.exit(1 if fails else 0)
