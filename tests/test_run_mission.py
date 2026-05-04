import unittest

from scripts import run_mission


class RunMissionConfigTests(unittest.TestCase):
    def test_px4_parameters_default_to_empty_mapping(self):
        parameters = run_mission.load_px4_parameters({"mission": {}})

        self.assertEqual(parameters, {})

    def test_px4_parameters_load_numeric_values(self):
        parameters = run_mission.load_px4_parameters(
            {
                "mission": {
                    "px4_parameters": {
                        "MPC_LAND_SPEED": 0.2,
                        "MPC_LAND_RC_HELP": 1,
                    }
                }
            }
        )

        self.assertEqual(
            parameters,
            {
                "MPC_LAND_SPEED": 0.2,
                "MPC_LAND_RC_HELP": 1,
            },
        )

    def test_px4_parameters_reject_non_mapping(self):
        with self.assertRaises(ValueError):
            run_mission.load_px4_parameters(
                {"mission": {"px4_parameters": ["MPC_LAND_SPEED"]}}
            )

    def test_px4_parameters_reject_non_numeric_value(self):
        with self.assertRaises(ValueError):
            run_mission.load_px4_parameters(
                {"mission": {"px4_parameters": {"MPC_LAND_SPEED": "slow"}}}
            )

    def test_landing_profile_defaults_to_standard(self):
        profile = run_mission.load_landing_profile({"mission": {}})

        self.assertEqual(profile, {"mode": "standard"})

    def test_landing_profile_loads_offboard_body_values(self):
        profile = run_mission.load_landing_profile(
            {
                "mission": {
                    "landing_profile": {
                        "mode": "offboard_body",
                        "forward_velocity_m_s": 2.0,
                        "right_velocity_m_s": 0.5,
                        "descent_rate_m_s": 0.25,
                        "yaw_rate_deg_s": 10.0,
                        "end_altitude_m": 2.0,
                        "duration_s": 30,
                        "timeout": 140,
                        "setpoint_interval_s": 0.2,
                    }
                }
            }
        )

        self.assertEqual(profile["mode"], "offboard_body")
        self.assertEqual(profile["forward_velocity_m_s"], 2.0)
        self.assertEqual(profile["right_velocity_m_s"], 0.5)
        self.assertEqual(profile["descent_rate_m_s"], 0.25)
        self.assertEqual(profile["yaw_rate_deg_s"], 10.0)
        self.assertEqual(profile["end_altitude_m"], 2.0)
        self.assertEqual(profile["duration_s"], 30.0)
        self.assertEqual(profile["timeout"], 140.0)
        self.assertEqual(profile["setpoint_interval_s"], 0.2)

    def test_landing_profile_loads_offboard_ned_values(self):
        profile = run_mission.load_landing_profile(
            {
                "mission": {
                    "landing_profile": {
                        "mode": "offboard_ned",
                        "north_velocity_m_s": 2.0,
                        "east_velocity_m_s": -0.5,
                        "descent_rate_m_s": 0.0,
                        "yaw_rate_deg_s": 10.0,
                        "duration_s": 25,
                    }
                }
            }
        )

        self.assertEqual(profile["mode"], "offboard_ned")
        self.assertEqual(profile["north_velocity_m_s"], 2.0)
        self.assertEqual(profile["east_velocity_m_s"], -0.5)
        self.assertEqual(profile["descent_rate_m_s"], 0.0)
        self.assertEqual(profile["yaw_rate_deg_s"], 10.0)
        self.assertEqual(profile["duration_s"], 25.0)

    def test_landing_profile_rejects_invalid_mode(self):
        with self.assertRaises(ValueError):
            run_mission.load_landing_profile(
                {"mission": {"landing_profile": {"mode": "sideways"}}}
            )

    def test_landing_profile_rejects_negative_descent_rate(self):
        with self.assertRaises(ValueError):
            run_mission.load_landing_profile(
                {
                    "mission": {
                        "landing_profile": {
                            "mode": "offboard_body",
                            "descent_rate_m_s": -0.1,
                        }
                    }
                }
            )

    def test_landing_profile_requires_completion_condition(self):
        with self.assertRaises(ValueError):
            run_mission.load_landing_profile(
                {
                    "mission": {
                        "landing_profile": {
                            "mode": "offboard_ned",
                            "north_velocity_m_s": 2.0,
                        }
                    }
                }
            )


if __name__ == "__main__":
    unittest.main()
