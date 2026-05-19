import random
import math
import time


def get_lidar_points():
    points = []
    current_time = time.time()

    base_targets = [
        {"angle": 30, "distance": 90},
        {"angle": 85, "distance": 140},
        {"angle": 150, "distance": 110},
        {"angle": 220, "distance": 180},
        {"angle": 300, "distance": 130}
    ]

    for target in base_targets:
        points.append({
            "angle": target["angle"] + random.randint(-3, 3),
            "distance": target["distance"] + random.randint(-5, 5)
        })

    for _ in range(25):
        points.append({
            "angle": random.randint(0, 359),
            "distance": random.randint(60, 250)
        })

    moving_angle = int((current_time * 35) % 360)
    moving_distance = int(130 + 40 * math.sin(current_time))

    points.append({
        "angle": moving_angle,
        "distance": moving_distance
    })

    return points
