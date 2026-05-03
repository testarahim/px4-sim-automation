import unittest

import numpy as np

from scripts import inspect_log_candidates


class InspectLogCandidateTests(unittest.TestCase):
    def test_detects_takeoff_and_landing_candidate(self):
        metrics = inspect_log_candidates.compute_candidate_metrics(
            np.array([0.0, 1.0, 2.0, 3.0, 4.0]),
            np.array([0.0, 0.2, 1.2, 1.5, 0.1]),
            ground_altitude_m=0.3,
            takeoff_altitude_m=1.0,
        )

        self.assertFalse(metrics["airborne_start"])
        self.assertTrue(metrics["starts_near_ground"])
        self.assertTrue(metrics["ends_near_ground"])
        self.assertTrue(metrics["takeoff_detected"])
        self.assertTrue(metrics["landing_detected"])
        self.assertEqual(metrics["takeoff_time_s"], 2.0)
        self.assertEqual(metrics["landing_time_s"], 4.0)

    def test_marks_airborne_start_as_weak_takeoff_candidate(self):
        metrics = inspect_log_candidates.compute_candidate_metrics(
            np.array([0.0, 1.0, 2.0]),
            np.array([1.2, 1.5, 0.2]),
            ground_altitude_m=0.3,
            takeoff_altitude_m=1.0,
        )

        self.assertTrue(metrics["airborne_start"])
        self.assertFalse(metrics["starts_near_ground"])
        self.assertFalse(metrics["takeoff_detected"])
        self.assertTrue(metrics["landing_detected"])
        self.assertIsNone(metrics["takeoff_time_s"])

    def test_negative_initial_altitude_is_not_near_ground(self):
        metrics = inspect_log_candidates.compute_candidate_metrics(
            np.array([0.0, 1.0, 2.0]),
            np.array([-2.0, 0.0, 1.2]),
            ground_altitude_m=0.3,
            takeoff_altitude_m=1.0,
        )

        self.assertFalse(metrics["starts_near_ground"])
        self.assertTrue(metrics["airborne_start"])
        self.assertFalse(metrics["takeoff_detected"])


if __name__ == "__main__":
    unittest.main()
