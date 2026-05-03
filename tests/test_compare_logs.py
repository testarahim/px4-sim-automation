import argparse
import unittest

import numpy as np

from scripts import compare_logs


class CompareLogAlignmentTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
