"""补光和倒装后，用 IDE 单独运行本文件。串口只打印诊断，不发送 HV。"""

import sensor
import time
import config as cfg
from harvest_camera import setup_camera, detect_frame, draw
from harvest_vision import HarvestVision

# 每轮改标签，保持白框在采样面内部。坐标按翻正后的 320×240 画面。
SAMPLE_LABEL = "near_green_high"
SAMPLE_ROI = (154, 114, 12, 12)
SAMPLE_PERIOD_MS = 750


def main():
    # 与正式识别共用相机盘上的 config.py，取样坐标按翻转后的画面。
    print("LAB probe orientation: HMIRROR=%s VFLIP=%s" % (cfg.HMIRROR, cfg.VFLIP))
    x, y, w, h = SAMPLE_ROI
    if x < 0 or y < 0 or w <= 0 or h <= 0 or x + w > 320 or y + h > 240:
        raise ValueError("SAMPLE_ROI must fit inside 320x240")
    setup_camera()
    judge = HarvestVision(cfg)
    last_sample = time.ticks_ms()
    while True:
        img = sensor.snapshot()
        greens, yellows, tree = detect_frame(img, judge)
        near, height, aligned, reason, target = judge.update(greens, yellows, tree)
        now = time.ticks_ms()
        if time.ticks_diff(now, last_sample) >= SAMPLE_PERIOD_MS:
            hist = img.get_histogram(roi=SAMPLE_ROI)
            low = hist.get_percentile(0.10)
            mid = hist.get_percentile(0.50)
            high = hist.get_percentile(0.90)
            print("%s LAB50=(%d,%d,%d) P10-P90 L=%d..%d A=%d..%d B=%d..%d" % (
                SAMPLE_LABEL, mid.l_value(), mid.a_value(), mid.b_value(),
                low.l_value(), high.l_value(), low.a_value(), high.a_value(),
                low.b_value(), high.b_value()))
            print("tree=%d near=%d height=%s aligned=%d reason=%s G=%s Y=%s" % (
                tree, near, height, aligned, reason,
                [blob[2:] for blob in greens], [blob[2:] for blob in yellows]))
            last_sample = now
        draw(img, greens, yellows, target, near, height, aligned, reason)
        img.draw_rectangle(SAMPLE_ROI, color=(255, 255, 255))


if __name__ == "__main__":
    main()
