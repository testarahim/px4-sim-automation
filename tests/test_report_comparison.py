import unittest

from scripts import report_comparison


class ReportComparisonTests(unittest.TestCase):
    def sample_metrics(self):
        return {
            "altitude_rmse": 4.0,
            "velocity_rmse": 2.0,
            "roll_rmse": 1.0,
            "pitch_rmse": 3.0,
            "yaw_rmse": 5.0,
            "evaluation": {
                "overall_pass": False,
                "altitude_pass": False,
                "velocity_pass": True,
                "attitude_pass": False,
            },
            "alignment": {
                "method": "takeoff",
                "takeoff_threshold_m": 1.5,
                "sim_time_offset_s": 9.0,
                "real_time_offset_s": 3.0,
            },
            "inputs": {
                "sim": "sim.ulg",
                "real": "real.ulg",
                "config": "scenario.yaml",
            },
            "segments": {
                "takeoff_climb": {
                    "sim": {"duration_s": 10.0},
                    "real": {"duration_s": 20.0},
                    "altitude_rmse": 1.0,
                    "velocity_rmse": 2.0,
                    "roll_rmse": 3.0,
                    "pitch_rmse": 4.0,
                    "yaw_rmse": 5.0,
                },
                "hover_cruise": {
                    "sim": {"duration_s": 5.0},
                    "real": {"duration_s": 6.0},
                    "altitude_rmse": 0.5,
                    "velocity_rmse": 1.5,
                    "roll_rmse": 2.5,
                    "pitch_rmse": 3.5,
                    "yaw_rmse": 4.5,
                },
                "landing": {
                    "sim": {"duration_s": 8.0},
                    "real": {"duration_s": 40.0},
                    "altitude_rmse": 7.0,
                    "velocity_rmse": 6.0,
                    "roll_rmse": 5.0,
                    "pitch_rmse": 4.0,
                    "yaw_rmse": 3.0,
                },
            },
        }

    def test_build_segment_rows_adds_duration_ratio(self):
        rows = report_comparison.build_segment_rows(self.sample_metrics())

        self.assertEqual(rows[0]["segment"], "takeoff_climb")
        self.assertEqual(rows[0]["real_to_sim_duration_ratio"], 2.0)
        self.assertEqual(rows[2]["segment"], "landing")
        self.assertEqual(rows[2]["real_to_sim_duration_ratio"], 5.0)

    def test_render_markdown_includes_findings(self):
        markdown = report_comparison.render_markdown(self.sample_metrics())

        self.assertIn("# Comparison Report", markdown)
        self.assertIn("| landing | 8.000 | 40.000 | 5.000 |", markdown)
        self.assertIn("Largest segment altitude RMSE: landing (7.000).", markdown)
        self.assertIn("Largest real/sim duration ratio: landing (5.000x).", markdown)

    def test_render_csv_includes_segment_rows(self):
        csv_text = report_comparison.render_csv(self.sample_metrics())

        self.assertIn("segment,sim_duration_s,real_duration_s", csv_text)
        self.assertIn("takeoff_climb,10.0,20.0,2.0", csv_text)
        self.assertIn("landing,8.0,40.0,5.0", csv_text)


if __name__ == "__main__":
    unittest.main()
