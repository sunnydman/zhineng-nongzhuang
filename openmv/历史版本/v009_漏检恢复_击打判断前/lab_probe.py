"""场地取样工具。在 OpenMV IDE 中单独运行。"""

import sensor
import time

# x, y, w, h，基于 320×240 原图。让框完整落在果身或枝条内部。
SAMPLE_ROI = (154, 114, 12, 12)
x, y, w, h = SAMPLE_ROI
if w <= 0 or h <= 0 or x < 0 or y < 0 or x + w > 320 or y + h > 240:
    raise ValueError("SAMPLE_ROI must fit inside 320x240")

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_exposure(True)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=2000)

last_sample = time.ticks_ms()
while True:
    frame = sensor.snapshot()
    now = time.ticks_ms()
    if time.ticks_diff(now, last_sample) >= 750:
        hist = frame.get_histogram(roi=SAMPLE_ROI)
        low = hist.get_percentile(0.10)
        mid = hist.get_percentile(0.50)
        high = hist.get_percentile(0.90)
        print("ROI=%s LAB50=(%d,%d,%d) P10-P90 L=%d..%d A=%d..%d B=%d..%d" % (
            SAMPLE_ROI, mid.l_value(), mid.a_value(), mid.b_value(),
            low.l_value(), high.l_value(), low.a_value(), high.a_value(),
            low.b_value(), high.b_value()))
        last_sample = now
    # 采样后再画框，避免标记进入颜色统计。
    frame.draw_rectangle(SAMPLE_ROI, color=(255, 255, 255))
