"""纯判定逻辑；不依赖sensor/pyb，可在电脑上验证。

色块统一为(x,y,w,h,cx,cy,pixels)，坐标均为整幅图像中的像素。
"""


def green_reject_reason(blob, cfg):
    """返回第一个未通过的门槛，供调试标注使用。"""
    x, y, w, h, cx, cy, pixels = blob
    area = w * h
    if w <= 0 or h <= 0 or pixels < cfg.MIN_PIXELS:
        return "SMALL"
    if area < cfg.MIN_AREA:
        return "SMALL"
    if area > cfg.MAX_AREA:
        return "BIG"
    if pixels / area < cfg.MIN_FILL:
        return "FILL"
    if min(w, h) / max(w, h) < cfg.MIN_ASPECT:
        return "ASPECT"
    return None


def valid_green(blob, cfg):
    return green_reject_reason(blob, cfg) is None


def overlaps_yellow(green, yellows, margin):
    # 仅检查图像平面包围盒，不能证明推杆三维路径无碰撞。
    x, y, w, h = green[:4]
    for yellow in yellows:
        a, b, c, d = yellow[:4]
        if (x <= a + c + margin and x + w >= a - margin
                and y <= b + d + margin and y + h >= b - margin):
            return True
    return False


def choose_target(greens, yellows, cfg):
    # 黄果不做绿果的形状过滤，尽量保留被遮挡的黄果禁区。
    candidates = [b for b in greens if valid_green(b, cfg)]
    best = None
    for blob in candidates:
        if not overlaps_yellow(blob, yellows, cfg.YELLOW_MARGIN_PX):
            if best is None or blob[6] > best[6]:
                best = blob
    return best, candidates


class StabilityFilter:
    """相邻帧近邻计数，不是实例跟踪器；目标交叉仍可能串目标。"""

    def __init__(self, frames, distance):
        self.frames = max(1, frames)
        self.distance_sq = distance * distance
        self.reset()

    def reset(self):
        self.previous = None
        self.count = 0

    def update(self, target):
        if target is None:
            self.reset()
            return False
        center = (target[4], target[5])
        close = False
        if self.previous is not None:
            dx = center[0] - self.previous[0]
            dy = center[1] - self.previous[1]
            close = dx * dx + dy * dy <= self.distance_sq
        self.count = min(self.frames, self.count + 1) if close else 1
        self.previous = center
        return self.count >= self.frames
