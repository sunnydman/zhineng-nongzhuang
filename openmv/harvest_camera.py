"""作业与 LAB 标定共用的取景和颜色检测。"""

import sensor
import config as cfg
from harvest_vision import contains
from vision import green_reject_reason, valid_yellow, valid_green


def setup_camera(test_exposure_us=None):
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_hmirror(cfg.HMIRROR)
    sensor.set_vflip(cfg.VFLIP)
    # 旧版板载 config.py 没有此开关，首次复测也使用自动模式。
    auto_camera = getattr(cfg, "AUTO_CAMERA", True)
    if test_exposure_us is not None:
        # 台架比较同一灯和姿态下的曝光，避免自动增益补偿曝光变化。
        sensor.set_auto_gain(False, gain_db=0.0)
        sensor.set_auto_exposure(False, exposure_us=test_exposure_us)
        sensor.set_auto_whitebal(True)
        camera_mode = "TEST"
    elif auto_camera:
        sensor.set_auto_exposure(True)
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)
        camera_mode = "AUTO"
    else:
        sensor.set_auto_exposure(False, exposure_us=cfg.EXPOSURE_US)
        sensor.set_auto_gain(False, gain_db=cfg.GAIN_DB)
        sensor.set_auto_whitebal(False, rgb_gain_db=cfg.RGB_GAIN_DB)
        camera_mode = "MANUAL"
    sensor.skip_frames(time=cfg.WARMUP_MS)
    print("OPENMV camera=%s exposure_us=%s gain_db=%s rgb_gain_db=%s roi=%s" % (
        camera_mode, sensor.get_exposure_us(),
        sensor.get_gain_db(), sensor.get_rgb_gain_db(), cfg.ROI))


def detect(img, threshold, roi, min_pixels, min_area):
    blobs = img.find_blobs([threshold], roi=roi, pixels_threshold=min_pixels,
                           area_threshold=min_area, merge=False,
                           x_stride=2, y_stride=1)
    return [(b.x(), b.y(), b.w(), b.h(), b.cx(), b.cy(), b.pixels()) for b in blobs]


def yellow_rim(img, white):
    x, y, w, h, cx, cy, pixels = white
    pad = cfg.HARVEST_YELLOW_RIM_PAD_PX
    left, top = max(x - pad, 0), max(y - pad, 0)
    right, bottom = min(x + w + pad, img.width()), min(y + h + pad, img.height())
    roi = (left, top, right - left, bottom - top)
    yellow_parts = detect(img, cfg.YELLOW_LAB, roi, 12, 12)
    yellow_pixels = sum(part[6] for part in yellow_parts)
    if yellow_pixels < max(cfg.HARVEST_YELLOW_RIM_MIN_PIXELS,
                           pixels * cfg.HARVEST_YELLOW_RIM_MIN_RATIO):
        return None
    green_parts = detect(img, cfg.GREEN_LAB, roi, 12, 12)
    if sum(part[6] for part in green_parts) >= yellow_pixels:
        return None
    # 联合白色内核与黄皮外沿，按整颗保护，黄皮太少的依然报 WHITE。
    return (left, top, right - left, bottom - top, cx, cy, pixels + yellow_pixels)


def glare_candidates(img, detector):
    hot = detect(img, cfg.HARVEST_GLARE_LAB, cfg.ROI,
                 cfg.HARVEST_GLARE_MIN_PIXELS, cfg.HARVEST_GLARE_MIN_PIXELS)
    spots = []
    white_risk = []
    recovered_yellow = []
    for blob in hot:
        short = min(blob[2], blob[3])
        long_side = max(blob[2], blob[3])
        if not detector.in_near_region(blob) or short * 2 < long_side:
            continue
        if (blob[6] >= cfg.HARVEST_WHITE_BLOCK_PIXELS
                and short >= cfg.HARVEST_GLARE_MIN_SHORT_PX):
            yellow = yellow_rim(img, blob)
            if yellow is None:
                white_risk.append(blob)
            else:
                recovered_yellow.append(yellow)
        elif (blob[6] < cfg.HARVEST_WHITE_BLOCK_PIXELS
              and short >= 7 and detector.in_track_band(blob)):
            spots.append(blob)
    if not spots:
        return [], white_risk, recovered_yellow

    # 高亮与明亮绿皮用同一位图。生命周期限定在本次检测，主图和常规黄果阈值可继续使用。
    mask = img.binary([cfg.HARVEST_GLARE_SKIN_LAB, cfg.HARVEST_GLARE_LAB],
                      to_bitmap=True, copy=True)
    mask.close(1)
    mask.open(2)
    candidates = detect(mask, (1, 1), cfg.ROI, cfg.STRIKE_MIN_VISIBLE_PIXELS, cfg.HARVEST_ENTRY_AREA)
    recovered = []
    for blob in candidates:
        if (not detector.in_near_region(blob) or not detector.in_track_band(blob)
                or not valid_green(blob, cfg) or detector.is_tree(blob)
                or min(blob[2], blob[3]) < cfg.HARVEST_GLARE_MIN_SHORT_PX
                or min(blob[2], blob[3]) < max(blob[2], blob[3]) * cfg.HARVEST_FRUIT_MIN_ASPECT
                or not any(contains(blob[:4], spot) for spot in spots)):
            continue
        skin = detect(img, cfg.HARVEST_GLARE_SKIN_LAB, blob[:4], 60, 60)
        if sum(part[6] for part in skin) >= blob[6] * 0.45:
            recovered.append(blob)
    return recovered, white_risk, recovered_yellow


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
    del green_image
    yellows = detect(img, cfg.YELLOW_LAB, (0, 0, img.width(), img.height()),
                     cfg.YELLOW_MIN_PIXELS, cfg.YELLOW_MIN_AREA)
    yellows = [blob for blob in yellows if valid_yellow(blob, cfg)]
    if cfg.HARVEST_STOP_MODE:
        recovered, white_risk, recovered_yellow = glare_candidates(img, detector)
        yellows = [blob for blob in yellows if not any(
            contains(yellow[:4], blob) for yellow in recovered_yellow)] + recovered_yellow
    else:
        recovered, white_risk = [], []
    return greens, yellows, tree_visible, recovered, white_risk


def draw(img, greens, yellows, target, near, height, aligned, reason, recovered=(), white_risk=()):
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
    for blob in white_risk:
        img.draw_rectangle(blob[:4], color=(255, 200, 110))
        img.draw_string(blob[0], blob[1], "WHITE", color=(255, 200, 110))
    for blob in recovered:
        img.draw_rectangle(blob[:4], color=(0, 220, 220))
        img.draw_string(blob[0], blob[1], "GLARE", color=(0, 220, 220))
    if target is not None and reason == "OCC":
        img.draw_rectangle(target[:4], color=(255, 170, 0))
        img.draw_string(target[0], target[1], "OCC", color=(255, 170, 0))
    elif target is not None and (aligned or not cfg.HARVEST_STOP_MODE):
        img.draw_cross(target[4], target[5], color=(255, 255, 255), size=10)
    img.draw_string(0, 0, "HV %d %s %d %s" % (near, height, aligned, reason), color=(255, 255, 255))
