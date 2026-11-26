from vision import StabilityFilter, overlaps_yellow, valid_green
from strike_decision import StrikeDecision


def contains(region, blob):
    if region is None:
        return False
    x, y, w, h = region
    return x <= blob[4] < x + w and y <= blob[5] < y + h


class HarvestVision:
    def __init__(self, cfg):
        self.cfg = cfg
        self.tracker = StabilityFilter(cfg.STABLE_FRAMES, cfg.TRACK_DISTANCE_PX)
        self.judge = StrikeDecision(
            cfg.ROI, cfg.STRIKE_CONFIRM_FRAMES, cfg.STRIKE_MIN_SHORT_SIDE_PX,
            cfg.STRIKE_MIN_LONG_SIDE_PX, cfg.STRIKE_MAX_STEP_PX,
            cfg.STRIKE_MIN_PROJECTED_AREA_PX2, cfg.STRIKE_MIN_VISIBLE_PIXELS)
        self.previous_height = "N"
        self.last_green = None
        self.occluded_frames = 0

    def height(self, blob):
        high = contains(self.cfg.HARVEST_HIGH_NEAR, blob)
        low = contains(self.cfg.HARVEST_LOW_NEAR, blob)
        if high == low:
            return "N"
        return "H" if high else "L"

    def in_near_region(self, blob):
        return (contains(self.cfg.HARVEST_HIGH_NEAR, blob)
                or contains(self.cfg.HARVEST_LOW_NEAR, blob))

    def in_track_band(self, blob):
        return abs(blob[4] - self.cfg.HARVEST_CENTER_X_PX) <= self.cfg.HARVEST_TRACK_HALF_WIDTH_PX

    def is_near_yellow(self, blob):
        w, h, pixels = blob[2], blob[3], blob[6]
        return (w * h >= self.cfg.HARVEST_ENTRY_AREA
                and min(w, h) >= self.cfg.MIN_GREEN_SHORT_SIDE_PX
                and pixels >= w * h * self.cfg.MIN_FILL
                and self.in_near_region(blob))

    def is_tree(self, blob):
        w, h, pixels = blob[2], blob[3], blob[6]
        long_side = max(w, h)
        cfg = self.cfg
        return (contains(cfg.HARVEST_TREE_REGION, blob)
                and w * h >= cfg.HARVEST_TREE_MIN_AREA
                and pixels >= cfg.HARVEST_TREE_MIN_PIXELS
                and long_side >= cfg.HARVEST_TREE_MIN_LONG
                and (min(w, h) <= long_side * cfg.HARVEST_TREE_MAX_ASPECT
                     or w * h >= cfg.HARVEST_TREE_VERY_LARGE))

    def close_size_ok(self, blob):
        height = self.height(blob)
        if height == "N":
            return False
        limits = self.cfg.HARVEST_HIGH_CLOSE_MIN if height == "H" else self.cfg.HARVEST_LOW_CLOSE_MIN
        if limits is None:
            return False
        w, h, pixels = blob[2], blob[3], blob[6]
        short, long_side, area, visible = limits
        return min(w, h) >= short and max(w, h) >= long_side and w * h >= area and pixels >= visible

    def merged_over(self, blob, previous):
        x, y, w, h = blob[:4]
        px, py, pw, ph = previous[:4]
        shared_w = min(x + w, px + pw) - max(x, px)
        shared_h = min(y + h, py + ph) - max(y, py)
        return (shared_w > 0 and shared_h > 0 and
                shared_w * shared_h * 2 >= pw * ph and w * h * 2 >= 3 * pw * ph)

    def reset_tracking(self):
        self.previous_height = "N"
        self.tracker.reset()
        self.judge.reset()

    def same_position(self, first, second):
        dx = first[4] - second[4]
        dy = first[5] - second[5]
        return dx * dx + dy * dy <= self.cfg.STRIKE_MAX_STEP_PX ** 2

    def update(self, greens, yellows, tree_visible=False, recovered=(), white_risk=()):
        cfg = self.cfg
        near_greens = [blob for blob in greens
                       if valid_green(blob, cfg) and blob[2] * blob[3] >= cfg.HARVEST_ENTRY_AREA
                       and self.in_near_region(blob) and not self.is_tree(blob)]
        near = (tree_visible or bool(near_greens) or bool(white_risk)
                or any(self.is_near_yellow(blob) for blob in yellows))
        had_green = bool(near_greens)
        if cfg.HARVEST_STOP_MODE:
            near_greens = [blob for blob in near_greens
                           if self.in_track_band(blob)
                           and min(blob[2], blob[3]) >= max(blob[2], blob[3]) * cfg.HARVEST_FRUIT_MIN_ASPECT]
            near_calibrated = (cfg.HARVEST_HIGH_CLOSE_MIN is not None and
                               cfg.HARVEST_LOW_CLOSE_MIN is not None)
            if near_calibrated:
                # 后排小块仍可作为视觉观测，出杆的候选计数只用近距合格块。
                close_greens = [blob for blob in near_greens
                                if self.height(blob) == "N" or self.close_size_ok(blob)]
                only_far = bool(near_greens) and not close_greens
                near_greens = close_greens
            else:
                only_far = False
            # 修复框可以解释反光粘连。独立绿果缺失时，只报告观察结果。
            if recovered and (len(recovered) != 1 or not any(
                    self.same_position(blob, recovered[0]) and not self.merged_over(blob, recovered[0])
                    for blob in near_greens)):
                self.last_green = None
                self.occluded_frames = 0
                self.reset_tracking()
                return True, "N", False, "GLARE", None
            previous = self.last_green
            if previous is not None:
                # 原色块被树吞并且没有独立近果时，旧框只显示，不能用于对准。
                limit = getattr(cfg, "HARVEST_OCCLUSION_FRAMES", 3)
                nearby = any(self.same_position(blob, previous) for blob in near_greens)
                merged = not nearby and any(
                    self.is_tree(blob) and self.merged_over(blob, previous) for blob in greens)
                if merged and self.occluded_frames < limit:
                    self.occluded_frames += 1
                    self.reset_tracking()
                    return True, "N", False, "OCC", previous
                self.last_green = None
                self.occluded_frames = 0
            if only_far:
                self.reset_tracking()
                return near, "N", False, "FAR", None
            if not near_greens and had_green:
                self.reset_tracking()
                return near, "N", False, "SHAPE_SIDE", None
            if not near_greens and white_risk:
                self.reset_tracking()
                return near, "N", False, "WHITE", None
        target = near_greens[0] if len(near_greens) == 1 else None
        height = self.height(target) if target is not None else "N"
        if height == "N":
            target = None
        if height != self.previous_height:
            self.tracker.reset()
            self.judge.reset()
        self.previous_height = height
        white_blocked = target is not None and overlaps_yellow(target, white_risk, cfg.STRIKE_YELLOW_MARGIN_PX)
        blocked = white_blocked or (
            target is not None and overlaps_yellow(target, yellows, cfg.STRIKE_YELLOW_MARGIN_PX))
        stable = self.tracker.update(None if blocked else target)
        reason = self.judge.update("T" if stable else "W", target, len(near_greens), blocked)
        if reason != "ALLOW":
            return near, "N", False, "WHITE" if white_blocked else reason, target
        if cfg.HARVEST_STOP_MODE:
            limits = cfg.HARVEST_HIGH_CLOSE_MIN if height == "H" else cfg.HARVEST_LOW_CLOSE_MIN
            if limits is None:
                self.last_green = target
                return near, height, False, "CAL", target
            if not self.close_size_ok(target):
                return near, height, False, "FAR", target
            if not tree_visible:
                return near, height, False, "NO_TREE", target
            self.last_green = target
            aligned = abs(target[4] - cfg.HARVEST_CENTER_X_PX) <= cfg.HARVEST_CENTER_TOLERANCE_PX
            return near, height, aligned, "AXIS" if aligned else "CENTER", target
        fire = cfg.HARVEST_HIGH_FIRE if height == "H" else cfg.HARVEST_LOW_FIRE
        return near, height, contains(fire, target), reason, target
