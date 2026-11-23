"""电脑运行的聚焦测试，不代表OpenMV实机识别通过。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "当前版本"))
import config as cfg
from vision import choose_target, green_reject_reason, overlaps_yellow, valid_yellow, StabilityFilter
from protocol import encode


class LogicTests(unittest.TestCase):
    def test_yellow_blocks_larger_green(self):
        near = (35, 35, 20, 20, 45, 45, 300)
        far = (100, 100, 20, 20, 110, 110, 200)
        yellow = (57, 35, 20, 20, 67, 45, 200)
        target, _ = choose_target([near, far], [yellow], cfg)
        self.assertEqual(target, far)
        self.assertIsNone(choose_target([near], [yellow], cfg)[0])

    def test_thin_branch_rejected(self):
        branch = (40, 40, 5, 100, 42, 90, 350)
        self.assertIsNone(choose_target([branch], [], cfg)[0])
        self.assertIsNone(choose_target([], [], cfg)[0])

    def test_short_branch_tip_rejected(self):
        tip = (68, 32, 14, 13, 75, 38, 155)
        self.assertEqual(green_reject_reason(tip, cfg), "SMALL")
        self.assertIsNone(choose_target([tip], [], cfg)[0])
        compact_tip = (68, 32, 16, 16, 76, 40, 190)
        self.assertEqual(green_reject_reason(compact_tip, cfg), "SMALL")

    def test_yellow_diagonal_distance_preserves_close_protection(self):
        green = (82, 86, 21, 24, 92, 98, 350)
        diagonal = (116, 128, 27, 38, 129, 147, 650)  # 图四：差13px、18px。
        close = (113, 122, 27, 38, 126, 141, 650)
        level = (121, 90, 27, 38, 134, 109, 650)
        self.assertFalse(overlaps_yellow(green, [diagonal], 18))
        self.assertTrue(overlaps_yellow(green, [close], 18))
        self.assertTrue(overlaps_yellow(green, [level], 18))

    def test_yellow_strip_filter_preserves_compact_and_vertical_fragments(self):
        height = cfg.YELLOW_STRIP_MAX_HEIGHT_PX
        self.assertFalse(valid_yellow((0, 0, 3 * height, height, 0, 0, 60), cfg))
        self.assertTrue(valid_yellow((0, 0, 3 * height - 1, height, 0, 0, 60), cfg))
        self.assertTrue(valid_yellow((0, 0, height, 3 * height, 0, 0, 60), cfg))
        self.assertTrue(valid_yellow((0, 0, 4 * height, height + 1, 0, 0, 60), cfg))

    def test_clipped_green_cannot_beat_complete_fruit(self):
        fruit = (100, 100, 20, 20, 110, 110, 200)
        roi_x, roi_y, roi_w, roi_h = cfg.ROI
        for x, y in ((roi_x, 100), (100, roi_y), (roi_x + roi_w - 20, 100),
                     (100, roi_y + roi_h - 20)):
            with self.subTest(x=x, y=y):
                clipped = (x, y, 20, 20, x + 10, y + 10, 300)
                self.assertEqual(green_reject_reason(clipped, cfg), "EDGE")
                self.assertEqual(choose_target([clipped, fruit], [], cfg), (fruit, [fruit]))
        x = roi_x + cfg.GREEN_ROI_MARGIN_PX
        y = roi_y + cfg.GREEN_ROI_MARGIN_PX
        clear = (x, y, 20, 20, x + 10, y + 10, 300)
        self.assertEqual(choose_target([clear], [], cfg)[0], clear)

    def test_stability_resets_on_loss_and_jump(self):
        f = StabilityFilter(3, 25)
        fruit = (0, 0, 20, 20, 10, 10, 200)
        self.assertFalse(f.update(fruit))
        self.assertFalse(f.update(fruit))
        self.assertTrue(f.update(fruit))
        self.assertFalse(f.update(None))
        self.assertFalse(f.update(fruit))
        jump = (100, 100, 20, 20, 110, 110, 200)
        self.assertFalse(f.update(jump))
        self.assertEqual(f.count, 1)

    def test_one_gap_requires_fresh_match_and_repeated_loss_resets(self):
        f = StabilityFilter(3, 25, 1)
        fruit = (60, 60, 30, 40, 75, 80, 850)
        moved = (70, 62, 28, 37, 84, 80, 760)
        self.assertFalse(f.update(fruit))
        self.assertFalse(f.update(fruit))
        self.assertTrue(f.update(fruit))
        self.assertFalse(f.update(None))
        self.assertTrue(f.update(moved))
        self.assertFalse(f.update(None))
        self.assertFalse(f.update(None))
        self.assertFalse(f.update(moved))

    def test_reacquisition_requires_a_quarter_overlap_of_the_smaller_box(self):
        fruit = (60, 60, 20, 20, 70, 70, 250)
        for dx, expected_stable in ((15, True), (16, False), (20, False)):
            with self.subTest(dx=dx):
                tracker = StabilityFilter(3, 25, 1)
                for _ in range(3):
                    tracker.update(fruit)
                self.assertFalse(tracker.update(None))
                nearby = (60 + dx, 60, 20, 20, 70 + dx, 70, 250)
                self.assertEqual(tracker.update(nearby), expected_stable)
                if not expected_stable:
                    self.assertFalse(tracker.update(nearby))
                    self.assertTrue(tracker.update(nearby))

    def test_blocked_frame_and_size_jump_clear_confirmation(self):
        f = StabilityFilter(2, 25, 1)
        fruit = (60, 60, 30, 40, 75, 80, 850)
        smaller = (70, 75, 16, 16, 78, 83, 170)
        f.update(fruit)
        self.assertTrue(f.update(fruit))
        self.assertFalse(f.update(smaller))
        f.update(fruit)
        self.assertTrue(f.update(fruit))
        self.assertFalse(f.update(None, allow_gap=False))
        self.assertFalse(f.update(fruit))

    def test_packet_target_and_invalidation(self):
        fruit = (0, 0, 20, 20, 10, 10, 200)
        self.assertEqual(encode(1, 100, "T", fruit, 1, 0),
                         b"$MV1,1,100,T,10,10,20,20,200,1,0*4D\n")
        for state in ("U", "N", "W", "E"):
            fields = encode(1, 100, state, fruit, 1, 0).split(b"*")[0].split(b",")
            self.assertEqual(fields[4:9], [b"-1", b"-1", b"0", b"0", b"0"])


if __name__ == "__main__":
    unittest.main()
