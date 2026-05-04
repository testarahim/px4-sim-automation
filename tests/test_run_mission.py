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

    def test_motion_profile_defaults_to_none(self):
        profile = run_mission.load_motion_profile({"mission": {}})

        self.assertEqual(profile, {"mode": "none"})

    def test_motion_profile_loads_offboard_ned_displacement_with_duration(self):
        profile = run_mission.load_motion_profile(
            {
                "mission": {
                    "motion_profile": {
                        "mode": "offboard_ned",
                        "north_m": 30.0,
                        "east_m": 40.0,
                        "target_altitude_m": 12.0,
                        "duration_s": 25.0,
                        "yaw_deg": 90.0,
                    }
                }
            }
        )

        self.assertEqual(profile["mode"], "offboard_ned")
        self.assertEqual(profile["north_m"], 30.0)
        self.assertEqual(profile["east_m"], 40.0)
        self.assertEqual(profile["target_altitude_m"], 12.0)
        self.assertEqual(profile["duration_s"], 25.0)
        self.assertEqual(profile["horizontal_speed_m_s"], 2.0)
        self.assertEqual(profile["yaw_deg"], 90.0)

    def test_motion_profile_derives_duration_from_horizontal_speed(self):
        profile = run_mission.load_motion_profile(
            {
                "mission": {
                    "motion_profile": {
                        "mode": "offboard_ned",
                        "north_m": 3.0,
                        "east_m": 4.0,
                        "horizontal_speed_m_s": 2.5,
                    }
                }
            }
        )

        self.assertEqual(profile["duration_s"], 2.0)
        self.assertEqual(profile["horizontal_speed_m_s"], 2.5)

    def test_motion_profile_loads_multiple_legs(self):
        profile = run_mission.load_motion_profile(
            {
                "mission": {
                    "motion_profile": {
                        "mode": "offboard_ned",
                        "legs": [
                            {
                                "north_m": 10.0,
                                "east_m": -5.0,
                                "duration_s": 4.0,
                                "yaw_start_deg": 5.0,
                                "yaw_end_deg": 25.0,
                            },
                            {
                                "north_m": 0.0,
                                "east_m": 0.0,
                                "duration_s": 2.0,
                            },
                        ],
                    }
                }
            }
        )

        self.assertEqual(profile["mode"], "offboard_ned")
        self.assertEqual(len(profile["legs"]), 2)
        self.assertEqual(profile["legs"][0]["north_m"], 10.0)
        self.assertEqual(profile["legs"][0]["yaw_start_deg"], 5.0)
        self.assertEqual(profile["legs"][0]["yaw_end_deg"], 25.0)
        self.assertEqual(profile["legs"][1]["duration_s"], 2.0)

    def test_motion_profile_rejects_missing_duration_and_speed(self):
        with self.assertRaises(ValueError):
            run_mission.load_motion_profile(
                {
                    "mission": {
                        "motion_profile": {
                            "mode": "offboard_ned",
                            "north_m": 3.0,
                        }
                    }
                }
            )


if __name__ == "__main__":
    unittest.main()
