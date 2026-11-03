"""plc_client 시험: 가짜 MC 프로토콜 서버 + MockPlc 시나리오. 실행: python3 tests/test_plc_client.py"""
import os, socket, struct, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import plc_client as pc

# ---- 가짜 PLC 서버 (3E 바이너리 프레임을 규격대로 해석) ----
class FakeServer:
    def __init__(self):
        self.mem_bits, self.mem_words, self.requests = {}, {}, []
        self.srv = socket.socket(); self.srv.bind(("127.0.0.1", 0)); self.srv.listen(5)
        self.port = self.srv.getsockname()[1]
        self.drop_next = False
        threading.Thread(target=self.accept, daemon=True).start()
    def accept(self):
        while True:
            c, _ = self.srv.accept()
            threading.Thread(target=self.serve, args=(c,), daemon=True).start()
    def rx(self, c, n):
        b = b""
        while len(b) < n:
            d = c.recv(n - len(b))
            if not d: raise EOFError
            b += d
        return b
    def serve(self, c):
        try:
            while True:
                h = self.rx(c, 9)
                assert h[:7] == b"\x50\x00\x00\xff\xff\x03\x00", h
                body = self.rx(c, struct.unpack("<H", h[7:9])[0])
                if self.drop_next:
                    self.drop_next = False; c.close(); return
                timer, cmd, sub = struct.unpack("<HHH", body[:6])
                addr = int.from_bytes(body[6:9], "little"); code = body[9]
                pts = struct.unpack("<H", body[10:12])[0]; data = body[12:]
                self.requests.append((cmd, sub, code, addr, pts))
                out = b""
                if cmd == 0x0401 and sub == 1:
                    bits = [self.mem_bits.get((code, addr + i), 0) for i in range(pts)]
                    if len(bits) % 2: bits.append(0)
                    out = bytes((bits[i] << 4) | bits[i + 1] for i in range(0, len(bits), 2))
                elif cmd == 0x0401 and sub == 0:
                    out = b"".join(struct.pack("<H", self.mem_words.get((code, addr + i), 0)) for i in range(pts))
                elif cmd == 0x1401 and sub == 1:
                    for i in range(pts):
                        byte = data[i // 2]; v = (byte >> 4) if i % 2 == 0 else (byte & 15)
                        self.mem_bits[(code, addr + i)] = v
                elif cmd == 0x1401 and sub == 0:
                    for i in range(pts):
                        self.mem_words[(code, addr + i)] = struct.unpack("<H", data[2*i:2*i+2])[0]
                resp = struct.pack("<H", 2 + len(out)) + b"\x00\x00" + out
                c.sendall(b"\xd0\x00\x00\xff\xff\x03\x00" + resp)
        except (EOFError, OSError):
            pass

def check(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    if not cond: check.failed += 1
check.failed = 0

# 1) 실제 클라이언트 + 가짜 서버
fs = FakeServer(); p = pc.MelsecPlc("127.0.0.1", fs.port)
p.write_bits("M3", [1, 0, 1]); 
check("bit write/read M3..M5", p.read_bits("M3", 3) == [1, 0, 1])
check("M code/addr", (0x90, 3) in [(r[2], r[3]) for r in fs.requests])
p.write_words("D0", [7, 1234, 45, 300])
check("word write/read D0..D3", p.read_words("D0", 4) == [7, 1234, 45, 300])
check("D code 0xA8", all(r[2] in (0x90, 0xA8) for r in fs.requests))
p.write_bits("X0A", [1]); check("hex X device (X0A -> 10)", fs.mem_bits.get((0x9C, 10)) == 1)
p.read_bits("M0", 12); check("odd-length bit read(12)", True)
p.write_bits("M10", [1]); p.write_bits("M12", [1,1,1])
check("odd count write M12..M14", p.read_bits("M12", 3) == [1,1,1])
fs.drop_next = True
check("auto-reconnect after drop", p.read_words("D0", 1) == [7])
p.close()
dead = pc.MelsecPlc("127.0.0.1", 1, timeout=0.3)
try: dead.read_bits("M0", 1); check("unreachable raises PlcError", False)
except pc.PlcError: check("unreachable raises PlcError", True)

# 2) MockPlc 시나리오
def run(fail="", power=100, **kw):
    m = pc.MockPlc(fail=fail, step_seconds=0.1)
    t0 = time.time()
    r = pc.run_fire_sequence(m, 3, 91.4, 45, power, log=lambda s: None,
                             timeouts={"ready": 0.5, "start": 0.5, "done": 1.5, "poll": 0.02, **kw})
    return m, r, time.time() - t0

m, r, dt = run()
check("normal fire ok", r.ok and r.stage == "done")
check("D0..D3 = 3,914,45,100", m.read_words("D0", 4) == [3, 914, 45, 100])
check("M3/M4 cleared by PLC, M5 set", m.read_bits("M3", 3) == [0, 0, 1])
m, r, _ = run(power=99999); check("power clamped 3000", m.read_words("D3", 1) == [3000])
m, r, _ = run("notready");  check("notready -> fail stage ready", (not r.ok) and r.stage == "ready")
m, r, _ = run("nostart");   check("nostart -> fail stage start + request cancelled", (not r.ok) and r.stage == "start" and m.read_bits("M3", 2) == [0, 0])
m, r, _ = run("error");     check("error -> code 1", (not r.ok) and r.err_code == 1 and "전진" in r.message)
m, r, _ = run("timeout");   check("timeout -> stage run, cancelled", (not r.ok) and r.stage == "run" and m.read_bits("M3", 2) == [0, 0])
# 이미 M10 오류 상태면 시작도 안 함
m = pc.MockPlc(); m.write_bits("M10", [1]); m.write_words("D20", [4])
r = pc.run_fire_sequence(m, 1, 90, 45, 100, log=lambda s: None)
check("pre-existing error blocks, code 4 = 비상정지", (not r.ok) and r.err_code == 4 and "비상정지" in r.message and m.read_bits("M4", 1) == [0])
# 통신 실패
r = pc.run_fire_sequence(pc.MelsecPlc("127.0.0.1", 1, timeout=0.3), 1, 90, 45, 100, log=lambda s: None)
check("comm failure -> stage comm", (not r.ok) and r.stage == "comm")

print("FAILED:", check.failed); sys.exit(1 if check.failed else 0)
