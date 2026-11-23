import unittest

from strike_decision import StrikeDecision


class StrikeDecisionTests(unittest.TestCase):
    fruit = (120, 110, 26, 36, 133, 128, 620)

    def setUp(self):
        self.judge = StrikeDecision((48, 48, 224, 166), 4, 18, 26, 12)

    def confirm_fruit(self):
        results = [self.judge.update("T", self.fruit, 1, False) for _ in range(4)]
        self.assertEqual(results, ["WAIT", "WAIT", "WAIT", "ALLOW"])

    def test_requires_four_current_qualifying_frames(self):
        self.assertEqual(self.judge.update("W", self.fruit, 1, False), "TRACK")
        self.confirm_fruit()

    def test_each_veto_clears_confirmed_qualification(self):
        cases = (
            ("N", None, 0, False, "NONE"),
            ("W", self.fruit, 1, False, "TRACK"),
            ("T", self.fruit, 1, True, "YELLOW"),
            ("T", self.fruit, 2, False, "MULTI"),
            ("T", (47, 110, 26, 36, 60, 128, 620), 1, False, "ZONE"),
            ("T", (120, 110, 20, 20, 130, 120, 300), 1, False, "SIZE"),
        )
        for state, target, count, near_yellow, reason in cases:
            with self.subTest(reason=reason):
                self.judge.reset()
                self.confirm_fruit()
                self.assertEqual(self.judge.update(state, target, count, near_yellow), reason)
                self.confirm_fruit()

    def test_target_jump_or_large_size_change_restarts_count(self):
        for candidate in (
            (133, 110, 26, 36, 146, 128, 620),
            (125, 115, 18, 26, 134, 128, 375),
        ):
            with self.subTest(candidate=candidate):
                self.judge.reset()
                self.confirm_fruit()
                self.assertEqual(self.judge.update("T", candidate, 1, False), "WAIT")


if __name__ == "__main__":
    unittest.main()
