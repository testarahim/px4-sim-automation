import argparse
import unittest

import numpy as np

from scripts import compare_logs


class CompareLogAlignmentTests(unittest.TestCase):
    def test_circular_rmse_uses_shortest_yaw_difference(self):
        rmse = compare_logs.compute_circular_rmse_deg(
            np.array([179.0, -179.0, 10.0]),
            np.array([-179.0, 179.0, 20.0]),
        )

        expected = np.sqrt(np.mean(np.array([-2.0, 2.0, -10.0]) ** 2))
        self.assertAlmostEqual(rmse, expected)

    def test_yaw_alignment_can_use_circular_rmse(self):
        _, _, _, rmse, _ = compare_logs.align_and_compare(
            np.array([0.0, 1.0]),
            np.array([179.0, -179.0]),
            np.array([0.0, 1.0]),
            np.array([-179.0, 179.0]),
            compare_logs.compute_circular_rmse_deg,
        )

        self.assertEqual(rmse, 2.0)

    def test_heading_normalized_yaw_removes_constant_offset(self):
        metrics = compare_logs.compute_heading_normalized_yaw_metrics(
            np.array([90.0, 100.0, 110.0]),
            np.array([10.0, 20.0, 30.0]),
        )

        self.assertAlmostEqual(metrics["yaw_heading_offset_deg"], 80.0)
        self.assertAlmostEqual(metrics["yaw_heading_normalized_rmse"], 0.0)

    def test_heading_normalized_yaw_handles_wraparound_offset(self):
        metrics = compare_logs.compute_heading_normalized_yaw_metrics(
            np.array([-170.0, -160.0]),
            np.array([170.0, -180.0]),
        )

        self.assertAlmostEqual(metrics["yaw_heading_offset_deg"], 20.0)
        self.assertAlmostEqual(metrics["yaw_heading_normalized_rmse"], 0.0)

    def test_resolve_takeoff_alignment_uses_first_threshold_crossing(self):
        args = argparse.Namespace(alignment="takeoff", takeoff_threshold_m=1.0)

        alignment = compare_logs.resolve_alignment(
            args,
            {},
            np.array([0.0, 5.0, 6.0]),
            np.array([0.0, 0.9, 1.1]),
            np.array([0.0, 2.0, 3.0]),
            np.array([0.0, 1.0, 1.5]),
        )

        self.assertEqual(alignment["method"], "takeoff")
        self.assertEqual(alignment["takeoff_threshold_m"], 1.0)
        self.assertEqual(alignment["sim_time_offset_s"], 6.0)
        self.assertEqual(alignment["real_time_offset_s"], 2.0)

    def test_shifted_takeoff_alignment_can_compare_offset_series(self):
        alignment = {
            "method": "takeoff",
            "takeoff_threshold_m": 1.0,
            "sim_time_offset_s": 1.0,
            "real_time_offset_s": 2.0,
        }

        sim_t, sim_values, real_values, rmse, details = compare_logs.align_and_compare(
            compare_logs.shifted_time(
                np.array([0.0, 1.0, 2.0, 3.0]),
                alignment,
                "sim",
            ),
            np.array([0.0, 1.0, 2.0, 3.0]),
            compare_logs.shifted_time(
                np.array([0.0, 1.0, 2.0, 3.0, 4.0]),
                alignment,
                "real",
            ),
            np.array([0.0, 0.0, 1.0, 2.0, 3.0]),
        )

        self.assertTrue(np.array_equal(sim_t, np.array([-1.0, 0.0, 1.0, 2.0])))
        self.assertTrue(np.array_equal(sim_values, real_values))
        self.assertEqual(rmse, 0.0)
        self.assertEqual(details["sample_count"], 4)

    def test_detect_segments_uses_altitude_phase_boundaries(self):
        alignment = {
            "method": "takeoff",
            "takeoff_threshold_m": 0.5,
            "sim_time_offset_s": 0.0,
            "real_time_offset_s": 0.0,
        }
        config = {
            "mission": {
                "takeoff_altitude": 10.0,
                "altitude_tolerance": 1.0,
                "hover_time": 1.0,
            },
            "thresholds": {
                "max_landing_final_altitude_m": 0.3,
            },
        }

        segments = compare_logs.detect_segments(
            np.arange(0.0, 11.0),
            np.array(
                [0.0, 0.3, 1.5, 5.0, 9.0, 10.0, 10.0, 9.0, 5.0, 0.2, 0.0]
            ),
            config,
            alignment,
            "sim",
        )

        self.assertEqual(segments["takeoff_climb"]["start_s"], 2.0)
        self.assertEqual(segments["takeoff_climb"]["end_s"], 4.0)
        self.assertEqual(segments["hover_cruise"]["start_s"], 4.0)
        self.assertEqual(segments["hover_cruise"]["end_s"], 7.0)
        self.assertEqual(segments["landing"]["start_s"], 7.0)
        self.assertEqual(segments["landing"]["end_s"], 9.0)

    def test_compare_segment_series_normalizes_different_segment_durations(self):
        sim_segment = {"start_s": 10.0, "end_s": 20.0, "duration_s": 10.0}
        real_segment = {"start_s": 100.0, "end_s": 140.0, "duration_s": 40.0}

        rmse, details = compare_logs.compare_segment_series(
            np.array([10.0, 15.0, 20.0]),
            np.array([0.0, 1.0, 2.0]),
            sim_segment,
            np.array([100.0, 120.0, 140.0]),
            np.array([0.0, 1.0, 2.0]),
            real_segment,
        )

        self.assertEqual(rmse, 0.0)
        self.assertTrue(details["available"])
        self.assertEqual(details["sample_count"], 3)

    def test_compare_segment_series_can_use_circular_rmse(self):
        sim_segment = {"start_s": 0.0, "end_s": 1.0, "duration_s": 1.0}
        real_segment = {"start_s": 0.0, "end_s": 1.0, "duration_s": 1.0}

        rmse, details = compare_logs.compare_segment_series(
            np.array([0.0, 1.0]),
            np.array([179.0, -179.0]),
            sim_segment,
            np.array([0.0, 1.0]),
            np.array([-179.0, 179.0]),
            real_segment,
            compare_logs.compute_circular_rmse_deg,
        )

        self.assertEqual(rmse, 2.0)
        self.assertTrue(details["available"])

    def test_segment_heading_normalized_yaw_removes_constant_offset(self):
        segment = {"start_s": 0.0, "end_s": 2.0, "duration_s": 2.0}

        metrics = compare_logs.compare_segment_heading_normalized_yaw(
            np.array([0.0, 1.0, 2.0]),
            np.array([90.0, 100.0, 110.0]),
            segment,
            np.array([0.0, 1.0, 2.0]),
            np.array([10.0, 20.0, 30.0]),
            segment,
        )

        self.assertAlmostEqual(metrics["yaw_heading_offset_deg"], 80.0)
        self.assertAlmostEqual(metrics["yaw_heading_normalized_rmse"], 0.0)


if __name__ == "__main__":
    unittest.main()
