"""OpenMV v008：动态筛选和黄果邻近避让。在 OpenMV IDE 中运行。"""

import sensor
import time
import config as cfg
from vision import (choose_target, green_reject_reason, overlaps_yellow,
                    valid_yellow, StabilityFilter)
from protocol import encode


def setup_camera():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_hmirror(cfg.HMIRROR)
    sensor.set_vflip(cfg.VFLIP)
    # 旧版板载 config.py 没有此开关，首次复测也使用自动模式。
    auto_camera = getattr(cfg, "AUTO_CAMERA", True)
    if auto_camera:
        sensor.set_auto_exposure(True)
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)
    else:
        sensor.set_auto_exposure(False, exposure_us=cfg.EXPOSURE_US)
        sensor.set_auto_gain(False, gain_db=cfg.GAIN_DB)
        sensor.set_auto_whitebal(False, rgb_gain_db=cfg.RGB_GAIN_DB)
    sensor.skip_frames(time=cfg.WARMUP_MS)
    print("OPENMV v008: camera=%s exposure_us=%s gain_db=%s rgb_gain_db=%s roi=%s" % (
        "AUTO" if auto_camera else "MANUAL", sensor.get_exposure_us(),
        sensor.get_gain_db(), sensor.get_rgb_gain_db(), cfg.ROI))


def detect(img, threshold, roi, min_pixels, min_area):
    blobs = img.find_blobs([threshold], roi=roi, pixels_threshold=min_pixels,
                           area_threshold=min_area, merge=False,
                           x_stride=2, y_stride=1)
    return [(b.x(), b.y(), b.w(), b.h(), b.cx(), b.cy(), b.pixels()) for b in blobs]


def detect_frame(img):
    green_image = img
    green_threshold = cfg.GREEN_LAB
    radius = cfg.GREEN_OPEN_RADIUS_PX
    if radius > 0:
        # 单独生成位图，原始 RGB565 帧仍供黄果检测和 IDE 显示。
        green_image = img.binary([green_threshold], to_bitmap=True, copy=True)
        green_image.open(radius)
        green_threshold = (1, 1)
    greens = detect(green_image, green_threshold, cfg.ROI, cfg.MIN_PIXELS, cfg.MIN_AREA)
    yellows = detect(img, cfg.YELLOW_LAB, (0, 0, img.width(), img.height()),
                     cfg.YELLOW_MIN_PIXELS, cfg.YELLOW_MIN_AREA)
    return greens, [blob for blob in yellows if valid_yellow(blob, cfg)]


def draw(img, greens, candidates, yellows, target, state):
    img.draw_rectangle(cfg.ROI, color=(0, 128, 255))  # 蓝框：实际绿果检测区。
    if getattr(cfg, "DRAW_REJECTED_GREEN", True):
        for b in greens:
            reason = green_reject_reason(b, cfg)
            if reason is not None:
                color = (230, 80, 255)
                img.draw_rectangle(b[:4], color=color)
                img.draw_string(b[0], b[1], reason, color=color)
    for b in yellows:
        img.draw_rectangle(b[:4], color=(255, 255, 0))
    for b in candidates:
        blocked = overlaps_yellow(b, yellows, cfg.YELLOW_MARGIN_PX)
        color = (255, 0, 0) if blocked else (0, 255, 0)
        label = "BLOCK" if blocked else "G"
        if state == "W" and b is target:
            label = "WAIT"
        img.draw_rectangle(b[:4], color=color)
        img.draw_string(b[0], b[1], label, color=color)
    if state == "T":
        img.draw_cross(target[4], target[5], color=(255, 255, 255), size=10)
    img.draw_string(0, 0, "TEST %s RAW:%d G:%d Y:%d" % (
        state, len(greens), len(candidates), len(yellows)), color=(255, 255, 255))


def main():
    uart = None
    if cfg.UART_ENABLED:
        from pyb import UART
        uart = UART(cfg.UART_ID, cfg.UART_BAUD, bits=8, parity=None, stop=1,
                    timeout=10, timeout_char=2)
    tracker = StabilityFilter(cfg.STABLE_FRAMES, cfg.TRACK_DISTANCE_PX, cfg.TRACK_MISSED_FRAMES)
    clock = time.clock()
    seq = 0
    last_send = time.ticks_ms()
    last_print = last_send
    try:
        setup_camera()
        while True:
            clock.tick()
            # 记录采图调用前的板端单调时间，不是精确曝光时间，也不能跨MCU相减。
            captured = time.ticks_ms()
            img = sensor.snapshot()
            greens, yellows = detect_frame(img)
            target, candidates = choose_target(greens, yellows, cfg)
            # 黄果挡住上一目标时立即清除跟踪确认，包括本帧没分离出绿果的情况。
            allow_gap = not candidates and (tracker.previous is None or not overlaps_yellow(
                tracker.previous, yellows, cfg.YELLOW_MARGIN_PX))
            stable = tracker.update(target, allow_gap=allow_gap)
            # 初值直接参与判断。T仅指连续稳定的视觉候选，不表示标定/路径已验证。
            if target is None:
                state = "N"
            else:
                state = "T" if stable else "W"
            # 检测完成后再画框，防止标注污染第二次色块识别。
            if cfg.DRAW:
                draw(img, greens, candidates, yellows, target, state)
            now = time.ticks_ms()
            if time.ticks_diff(now, last_send) >= cfg.SEND_PERIOD_MS:
                packet = encode(seq, captured, state, target, len(candidates), len(yellows))
                if uart is not None:
                    written = uart.write(packet)
                    if written != len(packet):
                        raise OSError("UART short write")
                last_send = now
            if time.ticks_diff(now, last_print) >= cfg.PRINT_PERIOD_MS:
                print("state=%s fps=%.1f RAW=%d G=%d Y=%d target=%s" % (
                    state, clock.fps(), len(greens), len(candidates), len(yellows),
                    None if target is None else target[4:6]))
                last_print = now
            seq = (seq + 1) & 0xFFFF
    except Exception as exc:
        # 不重发上一目标；错误帧尽力发送，链路故障时由接收端超时失效。
        tracker.reset()
        if uart is not None:
            try:
                uart.write(encode(seq, time.ticks_ms(), "E", None, 0, 0))
            except Exception:
                pass
        print("vision stopped:", exc)
        raise


if __name__ == "__main__":
    main()
