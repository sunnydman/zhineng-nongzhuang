"""作业与 LAB 标定共用的取景和颜色检测。"""

import sensor
import config as cfg
from vision import green_reject_reason, valid_yellow


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
    print("OPENMV camera=%s exposure_us=%s gain_db=%s rgb_gain_db=%s roi=%s" % (
        "AUTO" if auto_camera else "MANUAL", sensor.get_exposure_us(),
        sensor.get_gain_db(), sensor.get_rgb_gain_db(), cfg.ROI))


def detect(img, threshold, roi, min_pixels, min_area):
    blobs = img.find_blobs([threshold], roi=roi, pixels_threshold=min_pixels,
                           area_threshold=min_area, merge=False,
                           x_stride=2, y_stride=1)
    return [(b.x(), b.y(), b.w(), b.h(), b.cx(), b.cy(), b.pixels()) for b in blobs]


def detect_frame(img, detector):
    green_image = img.binary([cfg.GREEN_LAB], to_bitmap=True, copy=True)
    # 开运算前先找大枝干，空枝也可以维持慢速。
    if cfg.TREE_LAB is None or cfg.TREE_LAB == cfg.GREEN_LAB:
        tree_blobs = detect(green_image, (1, 1), cfg.ROI, cfg.HARVEST_TREE_MIN_PIXELS,
                            cfg.HARVEST_TREE_MIN_AREA)
    else:
        tree_blobs = detect(img, cfg.TREE_LAB, cfg.ROI, cfg.HARVEST_TREE_MIN_PIXELS,
                            cfg.HARVEST_TREE_MIN_AREA)
    tree_visible = any(detector.is_tree(blob) for blob in tree_blobs)
    radius = cfg.GREEN_OPEN_RADIUS_PX
    if radius > 0:
        green_image.open(radius)
    greens = detect(green_image, (1, 1), cfg.ROI, cfg.MIN_PIXELS, cfg.MIN_AREA)
    yellows = detect(img, cfg.YELLOW_LAB, (0, 0, img.width(), img.height()),
                     cfg.YELLOW_MIN_PIXELS, cfg.YELLOW_MIN_AREA)
    return greens, [blob for blob in yellows if valid_yellow(blob, cfg)], tree_visible


def draw(img, greens, yellows, target, near, height, aligned, reason):
    for region in (cfg.HARVEST_HIGH_NEAR, cfg.HARVEST_LOW_NEAR):
        if region is not None:
            img.draw_rectangle(region, color=(0, 128, 255))
    if cfg.HARVEST_STOP_MODE:
        x = cfg.HARVEST_CENTER_X_PX
        tol = cfg.HARVEST_CENTER_TOLERANCE_PX
        img.draw_line(x, 0, x, img.height() - 1, color=(0, 220, 220))
        img.draw_rectangle((x - tol, 0, 2 * tol + 1, img.height()), color=(0, 128, 128))
    else:
        for region in (cfg.HARVEST_HIGH_FIRE, cfg.HARVEST_LOW_FIRE):
            if region is not None:
                img.draw_rectangle(region, color=(0, 220, 220))
    for blob in greens:
        rejection = green_reject_reason(blob, cfg)
        if rejection is None or cfg.DRAW_REJECTED_GREEN:
            color = (0, 255, 0) if rejection is None else (230, 80, 255)
            img.draw_rectangle(blob[:4], color=color)
            img.draw_string(blob[0], blob[1], rejection or "%d,%d" % blob[4:6], color=color)
    for blob in yellows:
        img.draw_rectangle(blob[:4], color=(255, 255, 0))
    if target is not None:
        img.draw_cross(target[4], target[5], color=(255, 255, 255), size=10)
    img.draw_string(0, 0, "HV %d %s %d %s" % (near, height, aligned, reason), color=(255, 255, 255))
