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

    def height(self, blob):
        high = contains(self.cfg.HARVEST_HIGH_NEAR, blob)
        low = contains(self.cfg.HARVEST_LOW_NEAR, blob)
        if high == low:
            return "N"
        return "H" if high else "L"

    def in_near_region(self, blob):
        return (contains(self.cfg.HARVEST_HIGH_NEAR, blob)
                or contains(self.cfg.HARVEST_LOW_NEAR, blob))

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

    def update(self, greens, yellows, tree_visible=False):
        cfg = self.cfg
        near_greens = [blob for blob in greens
                       if valid_green(blob, cfg) and blob[2] * blob[3] >= cfg.HARVEST_ENTRY_AREA
                       and self.in_near_region(blob) and not self.is_tree(blob)]
        near = tree_visible or bool(near_greens) or any(self.is_near_yellow(blob) for blob in yellows)
        target = near_greens[0] if len(near_greens) == 1 else None
        height = self.height(target) if target is not None else "N"
        if height == "N":
            target = None
        if height != self.previous_height:
            self.tracker.reset()
            self.judge.reset()
        self.previous_height = height
        blocked = target is not None and overlaps_yellow(target, yellows, cfg.STRIKE_YELLOW_MARGIN_PX)
        stable = self.tracker.update(None if blocked else target)
        reason = self.judge.update("T" if stable else "W", target, len(near_greens), blocked)
        if reason != "ALLOW":
            return near, "N", False, reason, target
        if cfg.HARVEST_STOP_MODE:
            limits = cfg.HARVEST_HIGH_CLOSE_MIN if height == "H" else cfg.HARVEST_LOW_CLOSE_MIN
            if limits is None:
                return near, height, False, "CAL", target
            w, h, pixels = target[2], target[3], target[6]
            short, long_side, area, visible = limits
            if (min(w, h) < short or max(w, h) < long_side or
                    w * h < area or pixels < visible):
                return near, height, False, "FAR", target
            if not tree_visible:
                return near, height, False, "NO_TREE", target
            aligned = abs(target[4] - cfg.HARVEST_CENTER_X_PX) <= cfg.HARVEST_CENTER_TOLERANCE_PX
            return near, height, aligned, "AXIS" if aligned else "CENTER", target
        fire = cfg.HARVEST_HIGH_FIRE if height == "H" else cfg.HARVEST_LOW_FIRE
        return near, height, contains(fire, target), reason, target
