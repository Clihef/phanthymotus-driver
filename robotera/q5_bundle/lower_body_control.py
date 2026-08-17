"""Guarded relative position control for the Q5 lower-body joints.

The four supported joints share the body HybridJointCommand publisher with the
arm and head cards.  Lower-body joints carry the robot, so this card only
accepts small deltas from fresh measured positions; it never supplies a static
whole-body pose or assumes that zero is a safe standing position.
"""

from __future__ import annotations

import math
import threading
import time

from body_command import get_router
from control_contract import q5_active_status, q5_is_control_ready
from joint_limits import JOINT_LIMITS, limits_for


CARD = "lower_body_control"
TYPE = "actuator"
TOPIC = "/wr1_controller/commands"

ACTION_DETAILS = {
    "adjust_ankle": {
        "joint_name": "ankle_joint",
        "field": "ankle_delta_rad",
        "title": "踝关节俯仰微调",
    },
    "adjust_knee": {
        "joint_name": "knee_joint",
        "field": "knee_delta_rad",
        "title": "膝关节俯仰微调",
    },
    "adjust_hip": {
        "joint_name": "hip_joint",
        "field": "hip_delta_rad",
        "title": "胯关节俯仰微调",
    },
    "adjust_waist_yaw": {
        "joint_name": "waist_yaw_joint",
        "field": "waist_yaw_delta_rad",
        "title": "腰部偏航微调",
    },
}
LOWER_BODY_JOINTS = tuple(detail["joint_name"] for detail in ACTION_DETAILS.values())
DESC = (
    "Q5 下半身控制：腰、胯、膝、踝四关节相对当前位置的小步微调。"
    "承重关节默认禁止硬件执行，需完成位置直控准备并显式启用硬件开关。"
)


def _failure(code: str, message: str, **details) -> dict:
    return {"ok": False, "state": "error", "code": code,
            "message": message, "details": details}


def _finite_number(value, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{field} must be finite")
    return value


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        del namespace
        self._client = client
        self._hardware_enable = bool(plugin_config.get("hardware_enable", False))
        self._max_delta = float(plugin_config.get("max_delta_rad", 0.030))
        self._max_step = float(plugin_config.get("max_step_rad", 0.005))
        self._publish_rate = float(plugin_config.get("publish_rate_hz", 20.0))
        self._hold_repetitions = int(plugin_config.get("hold_repetitions", 3))
        self._settle_tolerance = float(plugin_config.get("settle_tolerance_rad", 0.020))
        self._settle_timeout = float(plugin_config.get("settle_timeout_s", 1.5))
        if min(self._max_delta, self._max_step, self._publish_rate,
               self._settle_tolerance, self._settle_timeout) <= 0:
            raise ValueError("lower_body_control limits, rate, and timeout must be positive")
        if self._max_step > self._max_delta:
            raise ValueError("lower_body_control max_step_rad cannot exceed max_delta_rad")
        if self._hold_repetitions < 1:
            raise ValueError("lower_body_control hold_repetitions must be at least 1")
        missing_limits = [name for name in LOWER_BODY_JOINTS if name not in JOINT_LIMITS]
        if missing_limits:
            raise ValueError(f"Q5 URDF is missing lower-body limits: {missing_limits}")

        self._router = get_router(client, executor) if self._hardware_enable else None
        self._lock = threading.Lock()
        self._motion_stop = None
        self._motion_thread = None
        self._active_command = None
        self._last_result = None

    def get_tool(self) -> dict:
        delta_fields = {}
        for detail in ACTION_DETAILS.values():
            joint_name = detail["joint_name"]
            lower, upper = JOINT_LIMITS[joint_name]
            delta_fields[detail["field"]] = {
                "type": "number",
                "title": "相对当前角度 (rad)",
                "minimum": -self._max_delta,
                "maximum": self._max_delta,
                "multipleOf": 0.005,
                "default": 0.0,
                "description": (
                    f"相对实测当前位置增量，范围[-{self._max_delta:g},{self._max_delta:g}]rad，"
                    f"默认0不运动；目标仍须位于URDF硬限位[{lower:g},{upper:g}]rad。"
                ),
            }
        return {
            "name": CARD,
            "type": TYPE,
            "multiInstance": False,
            "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", *ACTION_DETAILS, "cancel", "info"],
                        "oneOf": [
                            {"const": "start", "title": "检查控制条件"},
                            *[{"const": action, "title": detail["title"]}
                              for action, detail in ACTION_DETAILS.items()],
                            {"const": "cancel", "title": "取消并保持当前角度"},
                            {"const": "info", "title": "查看状态"},
                        ],
                    },
                    **delta_fields,
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查硬件开关、ROS、控制模式与反馈。"},
                    **{
                        action: {
                            "params": [detail["field"]],
                            "description": (
                                f"{detail['title']}；相对当前位置最多±{self._max_delta:g}rad，"
                                f"默认0rad不运动。"
                            ),
                        }
                        for action, detail in ACTION_DETAILS.items()
                    },
                    "cancel": {"params": [], "description": "取消当前插补，并保持最新实测角度。"},
                    "info": {"params": [], "description": "查看当前运动、上次反馈和安全条件。"},
                },
            },
        }

    def _safety(self) -> dict:
        router_status = self._router.status() if self._router is not None else {
            "ros_publisher_available": False,
            "active_owner": None,
            "other_publishers": [],
            "same_name_publisher_count": 0,
            "topic": TOPIC,
        }
        snap = self._client.snapshot()
        router_status.update({
            "hardware_enable": self._hardware_enable,
            "control_mode": "guarded_relative_joint_position",
            "command_message": "xbot_common_interfaces/msg/HybridJointCommand",
            "position_control_prepared": bool(
                getattr(self._client, "q5_position_control_prepared", False)),
            "joint_state_fresh": bool(snap.get("fresh", False)),
            "available_joints": [name for name in LOWER_BODY_JOINTS
                                 if name in snap.get("joints", {})],
            "q5_fsm": q5_active_status(self._client),
            "limits": {
                "max_delta_rad": self._max_delta,
                "max_step_rad": self._max_step,
                "max_interpolation_speed_radps": self._max_step * self._publish_rate,
                "settle_tolerance_rad": self._settle_tolerance,
                "settle_timeout_s": self._settle_timeout,
                "joint_position_limits": limits_for(LOWER_BODY_JOINTS),
                "joint_names_source": "q5_model.urdf",
                "deployment_guardrails_vendor_certified": False,
            },
        })
        return router_status

    def _publish(self, joint_name: str, position: float) -> bool:
        return bool(self._router and self._router.publish({joint_name: position}))

    def _hold_position(self, joint_name: str, position: float | None) -> bool:
        if position is None:
            return False
        published = False
        for index in range(self._hold_repetitions):
            published = self._publish(joint_name, float(position)) or published
            if index + 1 < self._hold_repetitions:
                time.sleep(1.0 / self._publish_rate)
        return published

    def _hold_current(self, joint_name: str) -> bool:
        snap = self._client.snapshot()
        position = snap.get("joints", {}).get(joint_name)
        return self._hold_position(joint_name, position) if snap.get("fresh") else False

    def _validate_adjustment(self, action: str, value):
        detail = ACTION_DETAILS[action]
        joint_name = detail["joint_name"]
        field = detail["field"]
        status = self._safety()
        if not self._hardware_enable:
            return _failure(
                "LOWER_BODY_CONTROL_DISABLED",
                "Q5 lower-body hardware control is disabled; validate on a supported "
                "robot and set hardware_enable=true",
                status=status,
            )
        if not status.get("ros_publisher_available"):
            return _failure("ROS_UNAVAILABLE", "Q5 body command publisher is unavailable", status=status)
        if status.get("same_name_publisher_count", 0) > 1:
            return _failure(
                "DUPLICATE_BODY_PUBLISHER",
                "Multiple q5_body_command publishers are active on the body command topic",
                status=status,
            )
        if status.get("other_publishers"):
            return _failure(
                "BODY_COMMAND_CONFLICT",
                "Refusing lower-body motion while another node publishes body commands",
                status=status,
            )
        if not status["position_control_prepared"]:
            return _failure(
                "DIRECT_CONTROL_NOT_PREPARED",
                "Run q5_control_mode action=prepare_position_control before lower-body control",
                status=status,
            )
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready or q5_status.get("state") != 4:
            return _failure(
                "Q5_FSM_NOT_ACTIVE",
                "Q5 must remain fresh and ACTIVE during lower-body control",
                status={**status, "q5_fsm": q5_status},
            )
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return _failure(
                "JOINT_STATE_UNAVAILABLE",
                "Refusing lower-body control without fresh /joint_states",
                status=status,
            )
        current = snap.get("joints", {}).get(joint_name)
        if current is None:
            return _failure(
                "JOINT_UNAVAILABLE",
                "Requested lower-body joint is absent from /joint_states",
                joint_name=joint_name,
            )
        try:
            delta = _finite_number(value, field)
        except ValueError as exc:
            return _failure("INVALID_ARGUMENT", str(exc))
        if abs(delta) > self._max_delta:
            return _failure(
                "DELTA_LIMIT_EXCEEDED",
                "Requested relative movement exceeds the per-call deployment guardrail",
                joint_name=joint_name,
                max_delta_rad=self._max_delta,
                requested_delta_rad=delta,
            )
        target = float(current) + delta
        lower, upper = JOINT_LIMITS[joint_name]
        if target < lower or target > upper:
            return _failure(
                "LIMIT_EXCEEDED",
                "Computed target is outside the Q5 URDF joint limit",
                joint_name=joint_name,
                current_position_rad=float(current),
                requested_delta_rad=delta,
                target_position_rad=target,
                min_rad=lower,
                max_rad=upper,
            )
        duration_s = 0.0 if abs(delta) < 1e-12 else max(
            0.25, abs(delta) / (self._max_step * self._publish_rate))
        return {
            "action": action,
            "joint_name": joint_name,
            "field": field,
            "current_position_rad": float(current),
            "delta_rad": delta,
            "target_position_rad": target,
            "duration_s": duration_s,
            "feedback_before_ms": snap.get("received_at_ms"),
        }

    def _wait_for_feedback(self, command: dict, stop_event=None) -> dict:
        deadline = time.monotonic() + self._settle_timeout
        joint_name = command["joint_name"]
        target = command["target_position_rad"]
        before = command.get("feedback_before_ms")
        latest = None
        while time.monotonic() < deadline:
            if stop_event is not None and stop_event.is_set():
                return {
                    "verified": False,
                    "cancelled": True,
                    "actual_position_rad": latest,
                    "position_error_rad": None if latest is None else abs(latest - target),
                    "feedback_received_at_ms": None,
                }
            snap = self._client.snapshot()
            actual = snap.get("joints", {}).get(joint_name)
            received = snap.get("received_at_ms")
            is_new = received is not None and (before is None or received > before)
            if snap.get("fresh") and actual is not None:
                latest = float(actual)
                error = abs(latest - target)
                if is_new and error <= self._settle_tolerance:
                    return {
                        "verified": True,
                        "actual_position_rad": latest,
                        "position_error_rad": error,
                        "feedback_received_at_ms": received,
                    }
            time.sleep(0.02)
        return {
            "verified": False,
            "actual_position_rad": latest,
            "position_error_rad": None if latest is None else abs(latest - target),
            "feedback_received_at_ms": None,
        }

    def _finish(self, result: dict, stop_event):
        with self._lock:
            self._last_result = dict(result)
            if self._motion_stop is stop_event:
                self._motion_stop = None
                self._motion_thread = None
                self._active_command = None

    def _run_move(self, stop_event, command: dict):
        joint_name = command["joint_name"]
        current = command["current_position_rad"]
        target = command["target_position_rad"]
        duration_s = command["duration_s"]
        steps = max(
            int(math.ceil(abs(target - current) / self._max_step)),
            int(math.ceil(duration_s * self._publish_rate)),
            1,
        )
        published = False
        try:
            for index in range(1, steps + 1):
                if stop_event.is_set():
                    break
                position = current + (target - current) * index / steps
                published = self._publish(joint_name, position) or published
                stop_event.wait(duration_s / steps if duration_s else 0.0)

            if stop_event.is_set():
                held = self._hold_current(joint_name)
                result = {
                    "ok": True,
                    "state": "stopped",
                    "code": "CANCELLED",
                    "message": "Lower-body adjustment cancelled; latest measured position held",
                    "command": dict(command),
                    "feedback_verified": False,
                    "hold_command_published": held,
                }
            elif not published:
                result = _failure(
                    "PUBLISH_FAILED",
                    "Q5 lower-body command could not be published",
                    command=command,
                )
            else:
                self._hold_position(joint_name, target)
                feedback = self._wait_for_feedback(command, stop_event)
                if stop_event.is_set():
                    held = self._hold_current(joint_name)
                    result = {
                        "ok": True,
                        "state": "stopped",
                        "code": "CANCELLED",
                        "message": "Lower-body adjustment cancelled while waiting for feedback",
                        "command": dict(command),
                        "feedback_verified": False,
                        "feedback": feedback,
                        "hold_command_published": held,
                    }
                elif feedback["verified"]:
                    result = {
                        "ok": True,
                        "state": "succeeded",
                        "command": dict(command),
                        "feedback_verified": True,
                        "feedback": feedback,
                    }
                else:
                    result = _failure(
                        "FEEDBACK_TIMEOUT",
                        "Command was published but fresh target feedback was not verified before timeout",
                        command=command,
                        feedback=feedback,
                        settle_timeout_s=self._settle_timeout,
                        settle_tolerance_rad=self._settle_tolerance,
                    )
        except Exception as exc:
            result = _failure(
                "INTERNAL_ERROR",
                "Lower-body motion worker failed",
                exception=str(exc),
                command=command,
            )
        finally:
            if self._router is not None:
                self._router.release(CARD)
            if "result" not in locals():
                result = _failure("INTERNAL_ERROR", "Lower-body motion worker exited unexpectedly")
            self._finish(result, stop_event)

    def _cancel(self, reason: str) -> dict:
        with self._lock:
            stop_event = self._motion_stop
            thread = self._motion_thread
        if stop_event is None:
            return {"ok": True, "state": "stopped", "reason": reason,
                    "message": "No lower-body movement was active"}
        stop_event.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._lock:
            result = dict(self._last_result) if self._last_result else None
        return result or {"ok": True, "state": "stopping", "reason": reason}

    def start(self):
        if not self._hardware_enable:
            return {"state": "disabled", "safety": self._safety()}
        return {"state": "ready" if self._router.status()["ros_publisher_available"]
                else "unavailable", "safety": self._safety()}

    def stop(self):
        return self._cancel("driver_shutdown")

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action in ("cancel", "stop"):
            return self._cancel("command")
        if action == "info":
            with self._lock:
                active = dict(self._active_command) if self._active_command else None
                last_result = dict(self._last_result) if self._last_result else None
            return {
                "ok": True,
                "state": "moving" if active else "idle",
                "active_command": active,
                "last_result": last_result,
                "safety": self._safety(),
            }
        if action not in ACTION_DETAILS:
            return None

        field = ACTION_DETAILS[action]["field"]
        command = self._validate_adjustment(action, args.get(field, 0.0))
        if command.get("ok") is False:
            return command
        if command["duration_s"] == 0.0:
            return {
                "ok": True,
                "state": "succeeded",
                "command": command,
                "no_op": True,
                "feedback_verified": True,
                "message": "Requested delta is zero; no command was published",
            }
        if not self._router.acquire(CARD):
            return _failure(
                "COMMAND_IN_PROGRESS",
                "Another Q5 body card currently owns the command publisher",
                status=self._router.status(),
            )
        with self._lock:
            if self._motion_thread is not None and self._motion_thread.is_alive():
                self._router.release(CARD)
                return _failure(
                    "MOTION_IN_PROGRESS",
                    "A lower-body movement is already active; cancel it before another move",
                )
            stop_event = threading.Event()
            self._motion_stop = stop_event
            self._active_command = dict(command)
            thread = threading.Thread(
                target=self._run_move,
                args=(stop_event, command),
                daemon=True,
                name="q5_lower_body_control",
            )
            self._motion_thread = thread
            thread.start()

        # Return verified controller feedback for a normal MCP call. A
        # concurrent cancel request can still stop the worker through the
        # ThreadingHTTPServer while this call waits.
        thread.join(timeout=command["duration_s"] + self._settle_timeout + 2.0)
        with self._lock:
            result = dict(self._last_result) if self._last_result else None
        return result or {
            "ok": True,
            "state": "moving",
            "command": command,
            "feedback_verified": False,
        }


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
