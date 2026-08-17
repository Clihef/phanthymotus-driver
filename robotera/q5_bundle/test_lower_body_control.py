"""ROS-free contract tests for the guarded Q5 lower-body card."""

from __future__ import annotations

import unittest

import lower_body_control


POSITIONS = {
    "ankle_joint": 0.20,
    "knee_joint": -0.50,
    "hip_joint": 0.10,
    "waist_yaw_joint": 0.0,
}


class Client:
    q5_position_control_prepared = True

    def __init__(self, positions=None, received_at_ms=1000):
        self.positions = dict(POSITIONS if positions is None else positions)
        self.received_at_ms = received_at_ms

    def snapshot(self):
        return {
            "available": True,
            "fresh": True,
            "received_at_ms": self.received_at_ms,
            "joints": dict(self.positions),
        }

    def sensor_snapshot(self, name):
        if name == "robot_status":
            return {"available": True, "fresh": True, "state": 4}
        return {}

    def get_lifecycle_state(self):
        return "active"


def enabled_plugin(client=None, **config):
    plugin = lower_body_control.Plugin(
        {"hardware_enable": True, **config}, "test", None, client or Client())
    plugin._router.status = lambda: {
        "ros_publisher_available": True,
        "other_publishers": [],
        "same_name_publisher_count": 1,
        "active_owner": None,
        "topic": "/wr1_controller/commands",
    }
    return plugin


class LowerBodyControlTests(unittest.TestCase):
    def test_schema_covers_all_four_joints_with_safe_defaults(self):
        schema = lower_body_control.Plugin({}, "test", None, Client()).get_tool()["inputSchema"]
        expected = {
            "adjust_ankle": "ankle_delta_rad",
            "adjust_knee": "knee_delta_rad",
            "adjust_hip": "hip_delta_rad",
            "adjust_waist_yaw": "waist_yaw_delta_rad",
        }
        self.assertEqual(set(schema["properties"]["action"]["enum"]),
                         set(schema["x-action-params"]))
        for action, field in expected.items():
            prop = schema["properties"][field]
            self.assertEqual(prop["default"], 0.0)
            self.assertEqual((prop["minimum"], prop["maximum"]), (-0.03, 0.03))
            self.assertEqual(schema["x-action-params"][action]["params"], [field])

    def test_hardware_execution_is_disabled_by_default(self):
        plugin = lower_body_control.Plugin({}, "test", None, Client())
        self.assertEqual(plugin.dispatch("start", {})["state"], "disabled")
        result = plugin.dispatch("adjust_hip", {"hip_delta_rad": 0.01})
        self.assertEqual(result["code"], "LOWER_BODY_CONTROL_DISABLED")

    def test_relative_delta_uses_live_position(self):
        plugin = enabled_plugin()
        command = plugin._validate_adjustment("adjust_knee", 0.02)
        self.assertAlmostEqual(command["current_position_rad"], -0.50)
        self.assertAlmostEqual(command["target_position_rad"], -0.48)
        self.assertAlmostEqual(command["delta_rad"], 0.02)

    def test_delta_and_urdf_limits_are_both_enforced(self):
        plugin = enabled_plugin()
        self.assertEqual(
            plugin._validate_adjustment("adjust_knee", 0.031)["code"],
            "DELTA_LIMIT_EXCEEDED",
        )
        limited = enabled_plugin(Client({"ankle_joint": 1.60}))
        self.assertEqual(
            limited._validate_adjustment("adjust_ankle", 0.02)["code"],
            "LIMIT_EXCEEDED",
        )

    def test_zero_default_is_a_no_op(self):
        plugin = enabled_plugin()
        for args in ({"waist_yaw_delta_rad": 0.0}, {}):
            result = plugin.dispatch("adjust_waist_yaw", args)
            self.assertTrue(result["ok"])
            self.assertTrue(result["no_op"])
            self.assertTrue(result["feedback_verified"])

    def test_other_body_publisher_is_rejected(self):
        plugin = enabled_plugin()
        plugin._router.status = lambda: {
            "ros_publisher_available": True,
            "other_publishers": [{"node_name": "another_controller"}],
            "same_name_publisher_count": 1,
        }
        result = plugin._validate_adjustment("adjust_hip", 0.01)
        self.assertEqual(result["code"], "BODY_COMMAND_CONFLICT")

    def test_duplicate_shared_body_publisher_is_rejected(self):
        plugin = enabled_plugin()
        plugin._router.status = lambda: {
            "ros_publisher_available": True,
            "other_publishers": [],
            "same_name_publisher_count": 2,
        }
        result = plugin._validate_adjustment("adjust_hip", 0.01)
        self.assertEqual(result["code"], "DUPLICATE_BODY_PUBLISHER")

    def test_feedback_must_be_new_and_within_tolerance(self):
        client = Client({"waist_yaw_joint": 0.019}, received_at_ms=1001)
        plugin = enabled_plugin(
            client, settle_timeout_s=0.05, settle_tolerance_rad=0.005)
        feedback = plugin._wait_for_feedback({
            "joint_name": "waist_yaw_joint",
            "target_position_rad": 0.02,
            "feedback_before_ms": 1000,
        })
        self.assertTrue(feedback["verified"])
        self.assertAlmostEqual(feedback["position_error_rad"], 0.001)

    def test_dispatch_returns_verified_feedback_after_publishing(self):
        client = Client()
        plugin = enabled_plugin(
            client, max_step_rad=0.03, publish_rate_hz=1000.0,
            hold_repetitions=1, settle_timeout_s=0.05,
            settle_tolerance_rad=0.001,
        )

        def publish(positions):
            client.positions.update(positions)
            client.received_at_ms += 1
            return True

        plugin._router.publish = publish
        result = plugin.dispatch("adjust_hip", {"hip_delta_rad": 0.01})
        self.assertTrue(result["ok"])
        self.assertEqual(result["state"], "succeeded")
        self.assertTrue(result["feedback_verified"])
        self.assertAlmostEqual(result["feedback"]["actual_position_rad"], 0.11)


if __name__ == "__main__":
    unittest.main()
