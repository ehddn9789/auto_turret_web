import math
import time
import signal
import sys
import ydlidar
import json
from lidar_shared import write_json


PORT = "/dev/ttyUSB0"

_previous_points = {}

# LiDAR 실제 수집 설정
SCAN_FREQUENCY = 8.0
SAMPLE_RATE = 5

running = True
current_laser = None

# 웹 표시용 설정
WEB_MIN_DISTANCE_CM = 50
WEB_MAX_DISTANCE_CM = 300
WEB_ANGLE_STEP = 3
WEB_MAX_POINTS = 150
PUBLISH_INTERVAL = 0.15

def handle_exit(signum, frame):
    global running, current_laser

    print("[LIDAR READER] stop signal received")
    running = False

    try:
        if current_laser is not None:
            current_laser.turnOff()
            current_laser.disconnecting()
            print("[LIDAR READER] lidar stopped")
    except Exception as e:
        print("[LIDAR READER STOP ERROR]", e)

    sys.exit(0)


signal.signal(signal.SIGTERM, handle_exit)
signal.signal(signal.SIGINT, handle_exit)

def optimize_points(raw_points):
    filtered = {}

    for p in raw_points:
        angle = int(p["angle"])
        distance = int(p["distance"])

        if distance < WEB_MIN_DISTANCE_CM:
            continue

        if distance > WEB_MAX_DISTANCE_CM:
            continue

        angle_key = int(angle / WEB_ANGLE_STEP) * WEB_ANGLE_STEP

        if angle_key not in filtered:
            filtered[angle_key] = {
                "angle": angle_key,
                "distance": distance
            }
        else:
            if distance < filtered[angle_key]["distance"]:
                filtered[angle_key] = {
                    "angle": angle_key,
                    "distance": distance
                }

    global _previous_points

    global _previous_points

    changed_points = []

    for point in filtered.values():

        angle = point["angle"]
        distance = point["distance"]

        previous_distance = _previous_points.get(angle)

        # 처음 보는 각도는 무조건 통과
        if previous_distance is None:
            changed_points.append(point)

        # 거리 변화가 3cm 이상일 때만 통과
        elif abs(distance - previous_distance) >= 3:
            changed_points.append(point)

        _previous_points[angle] = distance

    # 변화가 너무 적으면 기존 데이터 유지
    if len(changed_points) < 10:
        points = list(filtered.values())
    else:
        points = changed_points

    # 가까운 물체 우선 선택
    points.sort(key=lambda x: x["distance"])

    # MAX_POINTS 개만 선택
    points = points[:WEB_MAX_POINTS]

    # 레이더 표시용으로 각도 정렬
    points.sort(key=lambda x: x["angle"])

    return points


def create_lidar():
    ydlidar.os_init()

    laser = ydlidar.CYdLidar()

    laser.setlidaropt(ydlidar.LidarPropSerialPort, PORT)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)

    laser.setlidaropt(ydlidar.LidarPropScanFrequency, SCAN_FREQUENCY)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, SAMPLE_RATE)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

    laser.setlidaropt(ydlidar.LidarPropFixedResolution, True)
    laser.setlidaropt(ydlidar.LidarPropReversion, False)
    laser.setlidaropt(ydlidar.LidarPropInverted, False)
    laser.setlidaropt(ydlidar.LidarPropAutoReconnect, True)

    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 10.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.05)

    return laser


def main():
    global current_laser

    while running:
        laser = None

        try:
            print("[LIDAR READER] starting")

            laser = create_lidar()
            current_laser = laser

            if not laser.initialize():
                print("[LIDAR READER] initialize failed")
                time.sleep(2)
                continue

            if not laser.turnOn():
                print("[LIDAR READER] turnOn failed")
                time.sleep(2)
                continue

            print("[LIDAR READER] started")

            scan = ydlidar.LaserScan()
            last_publish_time = 0

            while True:
                raw_points = []

                if laser.doProcessSimple(scan):
                    for p in scan.points:
                        angle = math.degrees(p.angle)

                        if angle < 0:
                            angle += 360

                        distance_cm = int(p.range * 100)

                        raw_points.append({
                            "angle": angle,
                            "distance": distance_cm
                        })

                    points = optimize_points(raw_points)

                    now = time.time()

                    if now - last_publish_time >= PUBLISH_INTERVAL:
                        points_json = json.dumps(
                            {"points": points},
                            separators=(",", ":")
                        )

                        write_json(points_json)
                        last_publish_time = now

                time.sleep(0.02)

        except Exception as e:
            print("[LIDAR READER ERROR]", e)
            time.sleep(2)

        finally:
            try:
                if laser:
                    laser.turnOff()
                    laser.disconnecting()
            except Exception:
                pass

            current_laser = None


if __name__ == "__main__":
    main()