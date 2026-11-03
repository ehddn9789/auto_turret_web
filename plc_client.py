"""
PLC(미쓰비시 Q04UDV) 통신 모듈.

라즈베리파이가 PLC의 내부 접점(M)과 데이터 레지스터(D)를 읽고 쓰는 데 쓴다.
PLC 내장 이더넷 포트의 MC 프로토콜(3E 프레임, 바이너리)을 직접 구현했기 때문에
pymcprotocol 같은 외부 라이브러리가 필요 없다.

PLC 쪽 설정 (GX Works2 > PLC Parameter > Built-in Ethernet Port Setting)
  - Communication Data Code: Binary
  - Open Setting: Protocol TCP, Open System MC Protocol, Host Station Port No. 1388 (16진수, = 10진수 5000)

신호 표 (PLC 래더와 반드시 일치해야 함)
  라즈베리파이 -> PLC : D0 목표 번호, D1 Pan 각도 x10, D2 Tilt 각도, D3 발사 세기(ms),
                       M3 조준 완료, M4 발사 요청 (M5 는 파이가 시작 전에 0으로 지움)
  PLC -> 라즈베리파이 : M0 시스템 준비, M1 자동 모드, M2 발사 준비, M5 발사 완료,
                       M10 오류, M11 진행 중, D20 오류 코드(1 전진 시간초과, 3 후진 시간초과, 4 비상정지)
  PLC 가 스스로 끄는 것 : M3, M4 (발사 종료나 오류 때)

PLC 가 없을 때를 위해 같은 인터페이스의 MockPlc 도 들어 있다.
"""

import socket
import struct
import threading
import time
from collections import namedtuple

# ---------------------------------------------------------------------------
# 설정값
# ---------------------------------------------------------------------------

DEVICE_CODE = {"M": 0x90, "X": 0x9C, "Y": 0x9D, "D": 0xA8}
HEX_DEVICES = {"X", "Y"}  # X, Y 번호는 16진수 (예: X0A), M, D 는 10진수

# 큐시리즈(Q/L) 3E 프레임 서브커맨드
CMD_READ = 0x0401
CMD_WRITE = 0x1401
SUB_WORD = 0x0000
SUB_BIT = 0x0001

ERROR_TEXT = {
    1: "장전(실린더 전진) 시간 초과 - 전진 센서(B2)가 2초 안에 안 켜짐",
    3: "복귀(실린더 후진) 시간 초과 - 후진 센서(B1)가 2초 안에 안 켜짐",
    4: "비상정지",
}

# 발사 절차 단계별 대기 시간(초)
TIMEOUTS = {
    "ready": 3.0,    # M2(발사 준비)가 켜지길 기다리는 시간
    "start": 3.0,    # M4 를 켠 뒤 PLC 가 시작했다는 신호(M11 또는 M5)가 올 때까지
    "done": 10.0,    # 시작한 뒤 M5(발사 완료)가 올 때까지 (정상 최대 2+3+2초 + 여유)
    "poll": 0.1,     # 상태를 읽는 간격
}

FireResult = namedtuple("FireResult", ["ok", "stage", "message", "err_code"])


class PlcError(Exception):
    """PLC 통신 실패(연결 불가, 응답 이상, 종료 코드 오류 등)."""


def _parse_device(name):
    name = name.strip().upper()
    prefix = name[0]
    if prefix not in DEVICE_CODE:
        raise ValueError(f"지원하지 않는 디바이스: {name}")
    number = int(name[1:], 16 if prefix in HEX_DEVICES else 10)
    return DEVICE_CODE[prefix], number


def _encode_bits(values):
    # 비트 단위 데이터는 점 하나가 4비트(니블). 첫 번째 점이 위쪽 니블, 홀수 개면 마지막 아래쪽은 0.
    out = bytearray()
    for i in range(0, len(values), 2):
        hi = 1 if values[i] else 0
        lo = 1 if (i + 1 < len(values) and values[i + 1]) else 0
        out.append((hi << 4) | lo)
    return bytes(out)


def _decode_bits(data, count):
    bits = []
    for byte in data:
        bits.append((byte >> 4) & 0x0F)
        bits.append(byte & 0x0F)
    return [1 if b else 0 for b in bits[:count]]


# ---------------------------------------------------------------------------
# 실제 PLC (MC 프로토콜 3E, 바이너리, TCP)
# ---------------------------------------------------------------------------

class MelsecPlc:
    name = "PLC"
    is_mock = False

    def __init__(self, host, port=5000, timeout=2.0, monitoring_timer=0x0010):
        self.host = host
        self.port = int(port)
        self.timeout = timeout
        self.monitoring_timer = monitoring_timer  # 250 ms 단위. 0x0010 = 4초
        self._sock = None
        self._lock = threading.Lock()

    # -- 연결 --------------------------------------------------------------

    def _connect_locked(self):
        self._close_locked()
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        sock.settimeout(self.timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = sock

    def _close_locked(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def connect(self):
        """연결을 시도하고 성공 여부를 돌려준다(예외를 던지지 않음)."""
        with self._lock:
            try:
                self._connect_locked()
                return True
            except OSError:
                self._close_locked()
                return False

    def close(self):
        with self._lock:
            self._close_locked()

    # -- 저수준 송수신 ------------------------------------------------------

    def _recv_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise OSError("연결이 끊어짐")
            buf.extend(chunk)
        return bytes(buf)

    def _frame(self, command, subcommand, dev_code, dev_num, points, data=b""):
        body = struct.pack("<HHH", self.monitoring_timer, command, subcommand)
        body += struct.pack("<I", dev_num)[:3] + bytes([dev_code])
        body += struct.pack("<H", points) + data
        header = b"\x50\x00" + b"\x00" + b"\xFF" + struct.pack("<H", 0x03FF) + b"\x00"
        header += struct.pack("<H", len(body))
        return header + body

    def _exchange(self, frame):
        # 끊어진 소켓이면 한 번만 다시 연결해서 재시도한다.
        # (읽기, 쓰기 모두 같은 값을 다시 쓰는 것이라 중복 실행돼도 안전하다)
        with self._lock:
            last_error = None
            for _ in range(2):
                try:
                    if self._sock is None:
                        self._connect_locked()
                    self._sock.sendall(frame)
                    header = self._recv_exact(9)
                    (length,) = struct.unpack("<H", header[7:9])
                    body = self._recv_exact(length)
                    break
                except OSError as e:
                    last_error = e
                    self._close_locked()
            else:
                raise PlcError(f"PLC 통신 실패({self.host}:{self.port}): {last_error}")

        if header[0:2] != b"\xD0\x00":
            raise PlcError("PLC 응답 형식 오류")
        if len(body) < 2:
            raise PlcError("PLC 응답이 너무 짧음")
        (end_code,) = struct.unpack("<H", body[:2])
        if end_code != 0:
            raise PlcError(f"PLC 오류 응답 (종료코드 0x{end_code:04X})")
        return body[2:]

    # -- 공개 API ----------------------------------------------------------

    def read_bits(self, head, count):
        code, num = _parse_device(head)
        data = self._exchange(self._frame(CMD_READ, SUB_BIT, code, num, count))
        bits = _decode_bits(data, count)
        if len(bits) != count:
            raise PlcError("PLC 응답 길이 오류(비트 읽기)")
        return bits

    def write_bits(self, head, values):
        code, num = _parse_device(head)
        values = list(values)
        self._exchange(
            self._frame(CMD_WRITE, SUB_BIT, code, num, len(values), _encode_bits(values))
        )

    def read_words(self, head, count):
        code, num = _parse_device(head)
        data = self._exchange(self._frame(CMD_READ, SUB_WORD, code, num, count))
        if len(data) != count * 2:
            raise PlcError("PLC 응답 길이 오류(워드 읽기)")
        return list(struct.unpack("<%dH" % count, data))

    def write_words(self, head, values):
        code, num = _parse_device(head)
        values = [int(v) & 0xFFFF for v in values]
        self._exchange(
            self._frame(
                CMD_WRITE, SUB_WORD, code, num, len(values),
                struct.pack("<%dH" % len(values), *values),
            )
        )


# ---------------------------------------------------------------------------
# 가짜 PLC (PLC 없이 전체 흐름을 시험할 때)
# ---------------------------------------------------------------------------

class MockPlc:
    """래더(발사 시퀀스)를 흉내 내는 가짜 PLC.

    fail 로 실패 상황을 만들어 시험할 수 있다.
      ""         정상
      "notready" M2(발사 준비)가 켜지지 않음
      "nostart"  발사 요청을 받아도 시작하지 않음 (공 감지 센서 꺼짐 등)
      "error"    장전 시간 초과(M10, D20=1)
      "timeout"  시작은 하지만 끝나지 않음
    """

    name = "MOCK"
    is_mock = True

    def __init__(self, fail="", step_seconds=0.4):
        self.fail = (fail or "").lower()
        self.step_seconds = step_seconds
        self._lock = threading.RLock()
        self._bits = {}
        self._words = {}
        self._set_bit("M0", 1)
        self._set_bit("M1", 1)
        self._set_bit("M2", 0 if self.fail == "notready" else 1)

    def connect(self):
        return True

    def close(self):
        pass

    def _set_bit(self, dev, v):
        self._bits[dev.upper()] = 1 if v else 0

    def _get_bit(self, dev):
        return self._bits.get(dev.upper(), 0)

    @staticmethod
    def _split(head):
        head = head.upper()
        return head[0], int(head[1:], 16 if head[0] in HEX_DEVICES else 10)

    def read_bits(self, head, count):
        prefix, n = self._split(head)
        with self._lock:
            return [self._get_bit(f"{prefix}{n + i}") for i in range(count)]

    def write_bits(self, head, values):
        prefix, n = self._split(head)
        with self._lock:
            for i, v in enumerate(values):
                self._set_bit(f"{prefix}{n + i}", v)
            if self._get_bit("M4") and self._get_bit("M3") and self._get_bit("M2"):
                if self.fail != "nostart" and not self._get_bit("M11"):
                    threading.Thread(target=self._run_sequence, daemon=True).start()

    def read_words(self, head, count):
        prefix, n = self._split(head)
        with self._lock:
            return [self._words.get(f"{prefix}{n + i}", 0) for i in range(count)]

    def write_words(self, head, values):
        prefix, n = self._split(head)
        with self._lock:
            for i, v in enumerate(values):
                self._words[f"{prefix}{n + i}"] = int(v) & 0xFFFF

    def _run_sequence(self):
        with self._lock:
            power_ms = self._words.get("D3", 0)
            self._set_bit("M2", 0)
            self._set_bit("M11", 1)
        time.sleep(self.step_seconds)  # 장전
        if self.fail == "error":
            with self._lock:
                self._set_bit("M10", 1)
                self._words["D20"] = 1
                self._set_bit("M11", 0)
                self._set_bit("M3", 0)
                self._set_bit("M4", 0)
            return
        time.sleep(min(max(power_ms, 1), 3000) / 1000.0)  # 발사
        if self.fail == "timeout":
            return
        time.sleep(self.step_seconds)  # 복귀
        with self._lock:
            self._set_bit("M5", 1)
            self._set_bit("M11", 0)
            self._set_bit("M3", 0)
            self._set_bit("M4", 0)
            self._set_bit("M2", 1)


# ---------------------------------------------------------------------------
# 상태 읽기와 발사 절차
# ---------------------------------------------------------------------------

def read_status(plc):
    """PLC 상태를 한 번에 읽어 딕셔너리로 돌려준다 (통신 실패 시 PlcError)."""
    bits = plc.read_bits("M0", 12)  # M0 ~ M11
    code = plc.read_words("D20", 1)[0]
    return {
        "system_ready": bits[0],
        "auto_mode": bits[1],
        "fire_ready": bits[2],
        "aim_ok": bits[3],
        "fire_request": bits[4],
        "fire_done": bits[5],
        "error": bits[10],
        "busy": bits[11],
        "error_code": code,
        "error_text": ERROR_TEXT.get(code, f"알 수 없는 오류 코드 {code}") if bits[10] else "",
    }


def _cancel_request(plc):
    """발사 요청을 거둔다. 이미 시작된 시퀀스는 PLC 가 알아서 끝낸다."""
    for dev in ("M4", "M3"):
        try:
            plc.write_bits(dev, [0])
        except PlcError:
            pass


def _fail(stage, message, err_code=None):
    return FireResult(False, stage, message, err_code)


def run_fire_sequence(plc, target, pan_deg, tilt, power_ms, log=print, timeouts=None):
    """조준이 끝난 뒤 PLC 에 발사를 요청하고 끝날 때까지 기다린다.

    순서: 상태 확인 -> M2 대기 -> D0~D3 쓰기 -> M5 지움 -> M3, M4 켬
          -> 시작 확인(M11 / M5) -> 완료 대기(M5 / M10)
    실패하면 발사 요청(M3, M4)을 거두고 이유를 FireResult 로 돌려준다.
    """
    t = dict(TIMEOUTS)
    if timeouts:
        t.update(timeouts)

    power = int(max(1, min(3000, round(power_ms))))
    pan_x10 = int(round(float(pan_deg) * 10))

    try:
        # 1) 오류 상태 확인
        st = read_status(plc)
        if st["error"]:
            return _fail("check", f"PLC 오류 상태: {st['error_text']} (Reset 버튼으로 해제)", st["error_code"])

        # 2) 발사 준비(M2) 대기
        deadline = time.time() + t["ready"]
        while not st["fire_ready"]:
            if time.time() >= deadline:
                if not st["auto_mode"]:
                    why = "자동 모드가 아님 (Start 버튼을 눌러야 함)"
                elif not st["system_ready"]:
                    why = "시스템 준비 안 됨 (비상정지가 눌렸는지 확인)"
                else:
                    why = "실린더가 후진 위치(B1)가 아니거나 이전 발사가 진행 중"
                return _fail("ready", f"PLC 발사 준비 안 됨: {why}")
            time.sleep(t["poll"])
            st = read_status(plc)
            if st["error"]:
                return _fail("check", f"PLC 오류 상태: {st['error_text']}", st["error_code"])

        # 3) 값 쓰기 + 요청
        plc.write_words("D0", [target, pan_x10, int(tilt), power])
        plc.write_bits("M5", [0])
        plc.write_bits("M3", [1])
        plc.write_bits("M4", [1])
        log(f"[PLC] 발사 요청 (TARGET {target}, PAN {pan_deg}, TILT {tilt}, POWER {power}ms)")

        # 4) PLC 가 시작했는지 확인
        started = False
        deadline = time.time() + t["start"]
        while time.time() < deadline:
            st = read_status(plc)
            if st["error"]:
                _cancel_request(plc)
                return _fail("start", f"PLC 오류: {st['error_text']}", st["error_code"])
            if st["busy"] or st["fire_done"]:
                started = True
                break
            time.sleep(t["poll"])
        if not started:
            _cancel_request(plc)
            return _fail("start", "PLC가 발사를 시작하지 않음 (공 감지 센서 S1/X0C가 꺼져 있는지 확인)")

        # 5) 완료 대기
        deadline = time.time() + t["done"]
        while True:
            if st["fire_done"]:
                return FireResult(True, "done", "발사 완료", None)
            if st["error"]:
                _cancel_request(plc)
                return _fail("run", f"PLC 오류: {st['error_text']}", st["error_code"])
            if time.time() >= deadline:
                _cancel_request(plc)
                return _fail("run", "PLC 발사 완료 신호(M5)를 받지 못함 (시간 초과)")
            time.sleep(t["poll"])
            st = read_status(plc)

    except PlcError as e:
        _cancel_request(plc)
        return _fail("comm", str(e))
