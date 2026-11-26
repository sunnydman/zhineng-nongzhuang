"""从 OpenMV 向 Mega 发送画面观测。"""


def encode(seq, capture_ms, state, target, green_count, yellow_count):
    if state not in ("U", "N", "W", "T", "E"):
        raise ValueError("invalid state")
    if state == "T":
        if target is None:
            raise ValueError("T requires target")
        cx, cy, w, h, pixels = target[4], target[5], target[2], target[3], target[6]
    else:
        cx, cy, w, h, pixels = -1, -1, 0, 0, 0
    payload = "MV1,%d,%d,%s,%d,%d,%d,%d,%d,%d,%d" % (
        seq, capture_ms, state, cx, cy, w, h, pixels, green_count, yellow_count)
    checksum = 0
    for value in payload.encode("ascii"):
        checksum ^= value
    return ("$%s*%02X\n" % (payload, checksum)).encode("ascii")


def encode_harvest(seq, near, target, aligned, stop_mode=False):
    payload = "%s,%d,%d,%s,%d" % ("HV2" if stop_mode else "HV1", seq, int(near), target, int(aligned))
    checksum = 0
    for value in payload.encode("ascii"):
        checksum ^= value
    return ("$%s*%02X\n" % (payload, checksum)).encode("ascii")
