"""台架用的击打资格判断，输入为本帧视觉观测，输出为判断状态。"""


class StrikeDecision:
    def __init__(self, region, frames, min_short_side, min_long_side, max_step):
        self.region = region
        self.frames = frames
        self.min_short_side = min_short_side
        self.min_long_side = min_long_side
        self.max_step_sq = max_step * max_step
        self.reset()

    def reset(self):
        self.previous = None
        self.count = 0

    def update(self, state, target, candidate_count, near_yellow):
        if near_yellow:
            self.reset()
            return "YELLOW"
        if candidate_count > 1:
            self.reset()
            return "MULTI"
        if state != "T" or target is None or candidate_count != 1:
            self.reset()
            return "TRACK" if state == "W" else "NONE"

        x, y, w, h, cx, cy, _ = target
        left, top, width, height = self.region
        if x < left or y < top or x + w > left + width or y + h > top + height:
            self.reset()
            return "ZONE"
        if min(w, h) < self.min_short_side or max(w, h) < self.min_long_side:
            self.reset()
            return "SIZE"

        same_target = False
        if self.previous is not None:
            dx = cx - self.previous[4]
            dy = cy - self.previous[5]
            previous_area = self.previous[2] * self.previous[3]
            area = w * h
            same_target = (dx * dx + dy * dy <= self.max_step_sq
                           and 3 * min(previous_area, area) >= 2 * max(previous_area, area))
        self.count = min(self.frames, self.count + 1) if same_target else 1
        self.previous = target
        return "ALLOW" if self.count >= self.frames else "WAIT"
