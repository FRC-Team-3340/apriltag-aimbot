from __future__ import annotations

import unittest

from tracking.apriltag_tracker import AprilTagAimController, VisionTarget


def make_target(
    tx_degrees: float,
    ty_degrees: float,
    *,
    has_target: bool = True,
    is_stale: bool = False,
) -> VisionTarget:
    return VisionTarget(
        has_target=has_target,
        tx_degrees=tx_degrees,
        ty_degrees=ty_degrees,
        tag_id=7,
        timestamp_seconds=0.0,
        is_stale=is_stale,
        pipeline_latency_ms=0.0,
        capture_latency_ms=0.0,
    )


class AprilTagAimControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = AprilTagAimController(
            x_k_p=0.02,
            x_k_d=0.0,
            x_tolerance_degrees=1.5,
            x_minimum_command=0.07,
            x_maximum_command=0.35,
            y_k_p=0.02,
            y_k_d=0.0,
            y_tolerance_degrees=1.5,
            y_minimum_command=0.07,
            y_maximum_command=0.35,
        )

    def test_invalid_target_stops_both_axes(self) -> None:
        result = self.controller.calculate(make_target(12.0, -8.0, has_target=False))

        self.assertFalse(result.has_target)
        self.assertEqual(result.x_axis.output, 0.0)
        self.assertEqual(result.y_axis.output, 0.0)

    def test_stale_target_stops_both_axes(self) -> None:
        result = self.controller.calculate(make_target(12.0, -8.0, is_stale=True))

        self.assertFalse(result.has_target)
        self.assertEqual(result.x_axis.output, 0.0)
        self.assertEqual(result.y_axis.output, 0.0)

    def test_within_tolerance_outputs_zero_for_both_axes(self) -> None:
        result = self.controller.calculate(make_target(1.0, -1.0))

        self.assertTrue(result.within_tolerance)
        self.assertEqual(result.x_axis.output, 0.0)
        self.assertEqual(result.y_axis.output, 0.0)

    def test_small_error_uses_minimum_command_on_each_axis(self) -> None:
        result = self.controller.calculate(make_target(2.0, -2.0))

        self.assertTrue(result.has_target)
        self.assertAlmostEqual(result.x_axis.output, 0.07)
        self.assertAlmostEqual(result.y_axis.output, -0.07)

    def test_large_errors_clamp_independently(self) -> None:
        result = self.controller.calculate(make_target(-50.0, 60.0))

        self.assertEqual(result.x_axis.output, -0.35)
        self.assertEqual(result.y_axis.output, 0.35)


if __name__ == "__main__":
    unittest.main()