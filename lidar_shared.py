from multiprocessing import shared_memory

SHM_NAME = "auto_turret_lidar"
SHM_SIZE = 65536


def write_json(json_string):
    data = json_string.encode("utf-8")

    if len(data) >= SHM_SIZE:
        data = b'{"points":[]}'

    try:
        shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
    except FileExistsError:
        shm = shared_memory.SharedMemory(name=SHM_NAME)

    shm.buf[:SHM_SIZE] = b"\x00" * SHM_SIZE
    shm.buf[:len(data)] = data
    shm.close()


def read_json():
    try:
        shm = shared_memory.SharedMemory(name=SHM_NAME)
        raw = bytes(shm.buf[:SHM_SIZE]).split(b"\x00", 1)[0]
        shm.close()

        if not raw:
            return '{"points":[]}'

        return raw.decode("utf-8")

    except Exception:
        return '{"points":[]}'