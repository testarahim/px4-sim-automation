import unittest

import numpy as np

from scripts import profile_real_log


class ProfileRealLogTests(unittest.TestCase):
    def test_compute_real_profile_extracts_motion_shape(self):
        time_s = np.array([0.0, 2.0, 4.0, 6.0, 8.0, 10.0])
        x_m = np.array([0.0, 0.0, 2.0, 6.0, 10.0, 10.0])
        y_m = np.array([0.0, 0.0, 1.0, 3.0, 5.0, 5.0])
        altitude_m = np.array([0.0, 1.2, 9.0, 10.0, 9.0, 0.1])
        vx_m_s = np.array([0.0, 0.0, 1.0, 2.0, 2.0, 0.0])
        vy_m_s = np.array([0.0, 0.0, 0.5, 1.0, 1.0, 0.0])
        vz_m_s = np.array([0.0, -1.0, -1.0, 0.0, 0.5, 1.0])

        profile = profile_real_log.compute_real_profile(
            time_s,
            x_m,
            y_m,
            altitude_m,
            vx_m_s,
            vy_m_s,
            vz_m_s,
            ground_altitude_m=0.3,
            takeoff_threshold_m=1.0,
        )

        self.assertTrue(profile["takeoff_detected"])
        self.assertTrue(profile["landing_detected"])
        self.assertEqual(profile["takeoff_time_s"], 2.0)
        self.assertEqual(profile["landing_time_s"], 10.0)
        self.assertAlmostEqual(profile["max_altitude_m"], 10.0)
        self.assertAlmostEqual(
            profile["cruise_displacement"]["north_m"],
            8.0,
        )
        self.assertAlmostEqual(
            profile["cruise_displacement"]["east_m"],
            4.0,
        )
        self.assertAlmostEqual(
            profile["cruise_displacement"]["distance_m"],
            np.hypot(8.0, 4.0),
        )
        self.assertAlmostEqual(
            profile["cruise_displacement"]["heading_deg"],
            26.565051,
            places=5,
        )

    def test_build_scenario_from_profile_adds_offboard_motion(self):
        profile = {
            "target_altitude_m": 12.3,
            "cruise_duration_s": 20.0,
            "cruise_horizontal_speed_mean_m_s": 2.0,
            "cruise_displacement": {
                "north_m": 30.0,
                "east_m": 40.0,
                "distance_m": 50.0,
                "heading_deg": 53.13,
            },
        }

        scenario = profile_real_log.build_scenario_from_profile(profile)
        mission = scenario["mission"]

        self.assertEqual(mission["takeoff_altitude"], 12.3)
        self.assertEqual(mission["hover_time"], 3.0)
        self.assertEqual(mission["landing_profile"]["mode"], "offboard_ned")
        self.assertAlmostEqual(
            mission["landing_profile"]["north_velocity_m_s"],
            1.2,
        )
        self.assertAlmostEqual(
            mission["landing_profile"]["east_velocity_m_s"],
            1.6,
        )
        self.assertEqual(mission["landing_profile"]["duration_s"], 25.0)

    def test_build_scenario_without_motion_uses_hover_only(self):
        profile = {
            "target_altitude_m": 8.0,
            "cruise_duration_s": 6.0,
            "cruise_horizontal_speed_mean_m_s": 0.0,
            "cruise_displacement": {
                "north_m": 0.0,
                "east_m": 0.0,
                "distance_m": 0.0,
                "heading_deg": None,
            },
        }

        scenario = profile_real_log.build_scenario_from_profile(profile)

        self.assertEqual(scenario["mission"]["takeoff_altitude"], 8.0)
        self.assertEqual(scenario["mission"]["hover_time"], 6.0)
        self.assertNotIn("landing_profile", scenario["mission"])

    def test_profile_marks_missing_landing_as_not_detected(self):
        profile = profile_real_log.compute_real_profile(
            np.array([0.0, 1.0, 2.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 2.0, 3.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, -1.0, 0.0]),
            ground_altitude_m=0.3,
            takeoff_threshold_m=1.0,
        )

        self.assertTrue(profile["takeoff_detected"])
        self.assertFalse(profile["landing_detected"])
        self.assertEqual(profile["landing_time_s"], 2.0)


if __name__ == "__main__":
    unittest.main()
