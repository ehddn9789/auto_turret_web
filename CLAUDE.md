# Auto Turret — 프로젝트 메모리

대학 졸업 프로젝트: 목표물 9개 중 선택하면 사전 설정된 pan/tilt/power로
터렛이 회전해 압축공기로 폼공을 발사하는 시스템. 실탄/화약 아님.

## 하드웨어 구성

| 부품 | 스펙 |
|---|---|
| 메인 보드 | Raspberry Pi 5 |
| LiDAR | YDLIDAR X4 Pro (YDLidar-SDK, USB-UART, CP210x 어댑터) |
| Pan 축 모터 | IG52GM+Encoder 04Type DC 기어모터, 감속비 1/12, 엔코더 있음 |
| Tilt 축 모터 | 동일 계열이지만 **엔코더 없는 일반 DC 모터** |
| 모터 드라이버 | BTS7960 (R_IS/L_IS 과전류 감지 핀 있음) |
| 마이크로컨트롤러 | Arduino UNO (Pan 축 전담) |
| PLC | 미쯔비씨, GX Works2, 래더(Ladder) 언어로 작성 예정 |

## 중요한 설계 결정 (반드시 지켜야 함)

- **Tilt 축은 실질적으로 없는 것으로 취급**: 터렛은 항상 최대 높이(고정)로
  준비, 목표물은 항상 최저 높이에 배치. 자동조준은 **Pan(좌우)만** LiDAR로
  처리한다. **카메라 추가 확정** — PC에서 웹캠 + AprilTag(tag36h11)로
  목표물(태그 ID 1~9번)을 인식해 좌우 각도를 라즈베리파이로 전송하는 방식.
  `app.py`에 `/camera/target-angle`(POST), `/camera/target-angles`(GET)
  라우트 있음. PC측 인식 프로그램(`camera_vision.py`)은 별도 관리, 이
  저장소에는 포함하지 않음.
- **실제 발사(에어밸브)는 PLC가 담당**한다. Arduino/RPi는 조준(Pan 각도
  이동)만 하고 발사 로직은 없음. 구조: `PLC(READY) → RPi → Arduino(조준,
  AIM_OK) → PLC(FIRE_REQUEST 받아 솔레노이드 발사)`.
- PLC I/O 예시(확정 아님, 실제 배선 시 재확인 필요): M0 SYSTEM_READY,
  M1 AUTO_MODE, M2 FIRE_READY, M3 AIM_OK, D0 TARGET_NO, D1 PAN_ANGLE,
  D2 TILT_ANGLE, D3 POWER.
- 물리적 비상정지는 PLC 쪽에서 모터 전원 자체를 차단하는 방식으로 이미
  확보되어 있음 (소프트웨어 STOP와는 별개).

## 소프트웨어 구조

```
lidar_reader.py (YDLidar SDK) → 공유메모리(lidar_shared.py) → app.py
                                                                  │
                                            Flask + Flask-SocketIO
                                                                  │
                                          templates/index.html (프론트)
```

- `app.py`: Flask 서버. `TEST_MODE = True`면 아두이노/라이더 없이 목업으로
  동작. 실제 배포는 `systemd`(`auto_turret.service`,
  `auto_turret_lidar.service`)로 라즈베리파이에서 `python3 app.py` 직접
  실행 (gunicorn 아님, `socketio.run(..., allow_unsafe_werkzeug=True)`).
- `state_lock`(threading.RLock)으로 전역 상태/시리얼 접근 동기화. 여러
  워커 쓰면 안 됨(물리적으로 하드웨어 하나뿐).
- 시리얼 읽기는 `arduino_reader_loop()` 백그라운드 스레드가 전담 (쓰기와
  읽기를 분리해서 하트비트/응답이 꼬이지 않게 함).
- 관리자 인증: Flask `session` 기반, `/settings/<n>`·`/reset-data`는
  `@admin_required`.
- 프론트엔드는 웹소켓(`state_update`, `lidar_update`, `log_update`)으로
  실시간 갱신. 폴링은 소켓 끊겼을 때만 폴백.
- 디자인: iOS 글래스 스타일(반투명 카드, 블러, 보라 계열 accent). 모바일
  화면(≤640px)에서는 레이더/3D HUD 빼고 나머지 패널은 기본 접힘
  (`.collapsible`, `togglePanel()`), 탭 바도 접힘(`tabsWrap`).

## 아두이노 ↔ 라즈베리파이 통신 프로토콜

RPi가 보내는 명령 (전부 `PAN:` 문자열 포함 위치 무관하게 파싱됨):
```
TARGET:1,PAN:90,TILT:45,POWER:300   # 프리셋 발사 조준
TARGET:AUTO,PAN:..,TILT:..,POWER:.. # LiDAR 자동조준
MOVE:PAN:90,TILT:45,POWER:300       # 조이스틱/순찰 (고빈도, 로그 안 남김)
MANUAL_FIRE:PAN:..,TILT:..,POWER:.. # 수동 발사
STOP
HOME                                 # 리미트 스위치까지 이동해 0도 보정
```

아두이노가 보내는 응답:
```
ARDUINO_READY   # 부팅 완료
ALIVE           # 1초마다 하트비트
TARGET_SET:xx   # 새 목표각 수신 (RPi 로그에는 안 남김, 너무 잦음)
AIM_OK:xx       # 목표각 도달
STOPPED / ZERO_SET / HOMED / HOMING_START
ERROR_BAD_VALUE:.. / OVERCURRENT_STOP / MOVE_TIMEOUT
```

## Arduino 스케치 (`DW_JJ_Arduino.ino`) 핵심 사항

- Pan 축만 제어. 비례제어(P제어, `SPEED_KP=3.0`)로 목표각 오차에 비례한
  속도, 가속 램프(`ACCEL_RATE=300`/초)로 출발 시 서서히 가속, 도착 시
  자연 감속.
- 안전장치: `MOVE_TIMEOUT_MS=4000`(오래 못 도달하면 정지), 과전류 감지
  (`CURRENT_THRESHOLD=800`, **실측 후 보정 필요**), 워치독(`<avr/wdt.h>`,
  2초).
- 인코더: A상(D2)+B상(D3) 둘 다 인터럽트 사용(쿼드러처). **주의:**
  `COUNT_MODE_MULTIPLIER=2.0`으로 되어 있는데, B상 인터럽트 추가로 분해능이
  올라갔으니 실측해서 `4.0`으로 조정해야 할 가능성 높음. `PAN:180` 보내서
  실제 회전량 확인할 것.
- 홈 포지션: `HOME` 명령으로 리미트 스위치(D4, INPUT_PULLUP)까지 이동 후
  0도 확정. 아직 자동 실행 안 됨(수동 명령으로만 트리거).
- 0도/180도 양쪽 리미트 스위치(안전정지 + 자동 재보정용)는 **아직 미적용,
  다음 단계로 보류 중**. 필요해지면 추가 예정.

## 남은 작업 (진행 순서)

1. 아두이노 PWM 모터 제어 안정화 (진행 중) — 실측 후 `COUNT_MODE_MULTIPLIER`,
   `CURRENT_THRESHOLD`, 가속/감속 계수 튜닝
2. PLC 래더 로직 작성 (GX Works2)
3. 아두이노 ↔ PLC 통신 연결 (배선 + 신호 프로토콜)
4. LiDAR 자동조준(Pan만) 실제 하드웨어 테스트 — 예전에 RPi5에서 폴링 방식
   때문에 반응이 느렸던 적 있음(현재 웹소켓으로 개선됨, 재확인 필요)
5. HMI 구현 (가장 마지막, 수동 조작 등 단순 기능만)

## 파일 위치

```
auto_turret_web/
├── app.py
├── templates/index.html
├── static/style.css
├── lidar_reader.py / lidar_shared.py / lidar_data.py
├── requirements.txt (flask, flask-socketio, gunicorn, pyserial)
└── DW_JJ_Arduino.ino  (별도 관리, 이 저장소엔 없을 수 있음)
```
