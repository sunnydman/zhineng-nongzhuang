"""纯判定逻辑；不依赖sensor/pyb，可在电脑上验证。

色块统一为(x,y,w,h,cx,cy,pixels)，坐标均为整幅图像中的像素。
"""


def green_reject_reason(blob, cfg):
    """返回第一个未通过的门槛，供调试标注使用。"""
    x, y, w, h, cx, cy, pixels = blob
    area = w * h
    if w <= 0 or h <= 0 or pixels < cfg.MIN_PIXELS:
        return "SMALL"
    if (area < cfg.MIN_AREA or min(w, h) < cfg.MIN_GREEN_SHORT_SIDE_PX
            or max(w, h) < cfg.MIN_GREEN_LONG_SIDE_PX):
        return "SMALL"
    margin = cfg.GREEN_ROI_MARGIN_PX
    if margin > 0:
        roi_x, roi_y, roi_w, roi_h = cfg.ROI
        if (x < roi_x + margin or y < roi_y + margin
                or x + w > roi_x + roi_w - margin
                or y + h > roi_y + roi_h - margin):
            return "EDGE"
    if area > cfg.MAX_AREA:
        return "BIG"
    if pixels / area < cfg.MIN_FILL:
        return "FILL"
    if min(w, h) / max(w, h) < cfg.MIN_ASPECT:
        return "ASPECT"
    return None


def valid_green(blob, cfg):
    return green_reject_reason(blob, cfg) is None


def valid_yellow(blob, cfg):
    # 平视截图中，场地上的黄色细横条会误触发黄果避让。
    w, h = blob[2:4]
    return h > cfg.YELLOW_STRIP_MAX_HEIGHT_PX or w < 3 * h


def overlaps_yellow(green, yellows, margin):
    # 使用图像平面两框的最近距离，斜对角不再按方形外扩计算。
    x, y, w, h = green[:4]
    for yellow in yellows:
        a, b, c, d = yellow[:4]
        dx = max(a - (x + w), x - (a + c), 0)
        dy = max(b - (y + h), y - (b + d), 0)
        if dx * dx + dy * dy <= margin * margin:
            return True
    return False


def choose_target(greens, yellows, cfg):
    # yellows 为检测阶段过滤后的黄色块，供黄果避让使用。
    candidates = [b for b in greens if valid_green(b, cfg)]
    best = None
    for blob in candidates:
        if not overlaps_yellow(blob, yellows, cfg.YELLOW_MARGIN_PX):
            if best is None or blob[6] > best[6]:
                best = blob
    return best, candidates


class StabilityFilter:
    """用位置和大小确认同一候选，已锁定时可跨短暂的漏检。"""

    def __init__(self, frames, distance, missed_frames=0):
        self.frames = max(1, frames)
        self.distance_sq = distance * distance
        self.missed_limit = missed_frames
        self.reset()

    def reset(self):
        self.previous = None
        self.count = 0
        self.missed = 0

    def update(self, target, allow_gap=True):
        if target is None:
            if allow_gap and self.count >= self.frames and self.missed < self.missed_limit:
                self.missed += 1
            else:
                self.reset()
            return False
        close = False
        if self.previous is not None:
            dx = target[4] - self.previous[4]
            dy = target[5] - self.previous[5]
            previous_area = self.previous[2] * self.previous[3]
            area = target[2] * target[3]
            # 允许同一果受反光而收缩，拒绝从整果跳到小枝尖后继承确认。
            close = (dx * dx + dy * dy <= self.distance_sq
                     and 2 * min(previous_area, area) >= max(previous_area, area))
            if close and self.missed:
                # 跨漏检恢复还要求两框明显重叠，防止附近的同尺寸枝块继承 T。
                x, y, w, h = target[:4]
                a, b, c, d = self.previous[:4]
                overlap_w = max(0, min(x + w, a + c) - max(x, a))
                overlap_h = max(0, min(y + h, b + d) - max(y, b))
                close = 4 * overlap_w * overlap_h >= min(previous_area, area)
        self.count = min(self.frames, self.count + 1) if close else 1
        self.previous = target
        self.missed = 0
        return self.count >= self.frames
