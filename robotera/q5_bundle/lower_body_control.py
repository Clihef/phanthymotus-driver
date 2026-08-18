"""Guarded absolute position control for the Q5 lower-body joints.

The four supported joints share the body HybridJointCommand publisher with the
arm and head cards. User-facing targets are absolute degrees and are converted
to radians only for the Q5 HybridJointCommand interface.
"""

from __future__ import annotations

import math
import threading
import time

from body_command import get_router
from control_contract import q5_active_status, q5_is_control_ready
from joint_limits import JOINT_LIMITS


CARD = "lower_body_control"
TYPE = "actuator"
TOPIC = "/wr1_controller/commands"

ACTION_DETAILS = {
    "set_ankle": {
        "joint_name": "ankle_joint",
        "field": "ankle_position_deg",
        "title": "设置踝关节角度",
    },
    "set_knee": {
        "joint_name": "knee_joint",
        "field": "knee_position_deg",
        "title": "设置膝关节角度",
    },
    "set_hip": {
        "joint_name": "hip_joint",
        "field": "hip_position_deg",
        "title": "设置胯关节角度",
    },
    "set_waist_yaw": {
        "joint_name": "waist_yaw_joint",
        "field": "waist_yaw_position_deg",
        "title": "设置腰部偏航角度",
    },
}
LOWER_BODY_JOINTS = tuple(detail["joint_name"] for detail in ACTION_DETAILS.values())
DESC = (
    "Q5 下半身控制：以角度制输入腰、胯、膝、踝四关节的绝对目标位置。"
    "执行前自动完成位置直控准备并切换至 ACTIVE，目标严格受 Q5 URDF 限位约束。"
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


def _command_view(command: dict) -> dict:
    """Return degree-only command data for MCP results and card status."""
    return {key: value for key, value in command.items()
            if not key.endswith("_rad_internal")}


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        del namespace
        self._client = client
        self._hardware_enable = bool(plugin_config.get("hardware_enable", False))
        self._max_step_deg = float(plugin_config.get("max_step_deg", 0.3))
        self._max_step = math.radians(self._max_step_deg)
        self._publish_rate = float(plugin_config.get("publish_rate_hz", 20.0))
        self._hold_repetitions = int(plugin_config.get("hold_repetitions", 3))
        self._settle_tolerance_deg = float(plugin_config.get("settle_tolerance_deg", 1.0))
        self._settle_tolerance = math.radians(self._settle_tolerance_deg)
        self._settle_timeout = float(plugin_config.get("settle_timeout_s", 1.5))
        if min(self._max_step, self._publish_rate,
               self._settle_tolerance, self._settle_timeout) <= 0:
            raise ValueError("lower_body_control limits, rate, and timeout must be positive")
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
        position_fields = {}
        for detail in ACTION_DETAILS.values():
            joint_name = detail["joint_name"]
            lower, upper = JOINT_LIMITS[joint_name]
            lower_deg = math.ceil(math.degrees(lower) * 100.0) / 100.0
            upper_deg = math.floor(math.degrees(upper) * 100.0) / 100.0
            position_fields[detail["field"]] = {
                "type": "number",
                "title": "目标绝对角度 (°)",
                "minimum": lower_deg,
                "maximum": upper_deg,
                "multipleOf": 0.1,
                "default": 0.0,
                "description": (
                    f"绝对关节位置，角度制；默认0°；Q5 URDF限位"
                    f"[{lower_deg:g}°, {upper_deg:g}°]。"
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
                    **position_fields,
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查硬件开关、ROS、控制模式与反馈。"},
                    **{
                        action: {
                            "params": [detail["field"]],
                            "description": (
                                f"{detail['title']}；输入绝对角度，默认0°，"
                                "执行前自动准备位置控制并进入ACTIVE。"
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
            "control_mode": "guarded_absolute_joint_position",
            "command_message": "xbot_common_interfaces/msg/HybridJointCommand",
            "position_control_prepared": bool(
                getattr(self._client, "q5_position_control_prepared", False)),
            "direct_joint_control_prepared": bool(getattr(
                self._client, "q5_direct_joint_control_prepared", False)),
            "joint_state_fresh": bool(snap.get("fresh", False)),
            "available_joints": [name for name in LOWER_BODY_JOINTS
                                 if name in snap.get("joints", {})],
            "q5_fsm": q5_active_status(self._client),
            "limits": {
                "max_step_deg": self._max_step_deg,
                "max_interpolation_speed_degps": self._max_step_deg * self._publish_rate,
                "settle_tolerance_deg": self._settle_tolerance_deg,
                "settle_timeout_s": self._settle_timeout,
                "joint_position_limits_deg": {
                    name: {
                        "min_deg": math.degrees(JOINT_LIMITS[name][0]),
                        "max_deg": math.degrees(JOINT_LIMITS[name][1]),
                    }
                    for name in LOWER_BODY_JOINTS
                },
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

    @staticmethod
    def _external_command_conflict(status: dict):
        # False means the endpoint remained silent for a complete observation
        # window. True means recent traffic; None means monitoring has not yet
        # established that the endpoint is quiet, so fail closed.
        active = [endpoint for endpoint in status.get("other_publishers", [])
                  if endpoint.get("actively_publishing") is not False]
        mpc_publishers = [endpoint for endpoint in active
                          if endpoint.get("node_name") == "mpc_policy_node"]
        if mpc_publishers:
            return _failure(
                "MPC_CONTROL_CONFLICT",
                "Remote-control/MPC command traffic must be quiet before direct joint-motor control",
                mpc_publishers=mpc_publishers,
                status=status,
            )
        if active:
            return _failure(
                "BODY_COMMAND_CONFLICT",
                "Another node is actively publishing body commands or has not yet been proven quiet",
                active_publishers=active,
                status=status,
            )
        return None

    def _validate_adjustment(self, action: str, value):
        detail = ACTION_DETAILS[action]
        joint_name = detail["joint_name"]
        field = detail["field"]
        try:
            target_deg = _finite_number(value, field)
        except ValueError as exc:
            return _failure("INVALID_ARGUMENT", str(exc))
        target = math.radians(target_deg)
        lower, upper = JOINT_LIMITS[joint_name]
        if target < lower or target > upper:
            return _failure(
                "LIMIT_EXCEEDED",
                "Requested absolute target is outside the Q5 URDF joint limit",
                joint_name=joint_name,
                target_position_deg=target_deg,
                min_deg=math.degrees(lower),
                max_deg=math.degrees(upper),
            )
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
        conflict = self._external_command_conflict(status)
        if conflict:
            return conflict
        preflight_snapshot = self._client.snapshot()
        if (not preflight_snapshot.get("fresh")
                or joint_name not in preflight_snapshot.get("joints", {})):
            return _failure(
                "JOINT_STATE_UNAVAILABLE",
                "Fresh target-joint feedback is required before automatic mode changes",
                joint_name=joint_name,
                status=status,
            )
        ensure_active = getattr(self._client, "ensure_q5_direct_joint_active", None)
        if not callable(ensure_active):
            return _failure(
                "CONTROL_MODE_AUTOMATION_UNAVAILABLE",
                "q5_control_mode did not register minimal direct-joint ACTIVE preparation",
                status=status,
            )
        preparation = ensure_active()
        if not isinstance(preparation, dict) or preparation.get("ok") is False:
            return _failure(
                "AUTO_PREPARE_FAILED",
                "Could not automatically prepare minimal direct-joint position control and ACTIVE state",
                preparation=preparation,
            )
        status = self._safety()
        conflict = self._external_command_conflict(status)
        if conflict:
            return conflict
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if (not status["direct_joint_control_prepared"] or not q5_ready
                or q5_status.get("state") != 4):
            return _failure(
                "Q5_FSM_NOT_ACTIVE",
                "Automatic preparation completed without fresh ACTIVE confirmation",
                status={**status, "q5_fsm": q5_status}, preparation=preparation,
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
        movement = target - float(current)
        duration_s = 0.0 if abs(movement) < 1e-12 else max(
            0.25, abs(movement) / (self._max_step * self._publish_rate))
        return {
            "action": action,
            "joint_name": joint_name,
            "field": field,
            "current_position_deg": math.degrees(float(current)),
            "target_position_deg": target_deg,
            "current_position_rad_internal": float(current),
            "target_position_rad_internal": target,
            "duration_s": duration_s,
            "feedback_before_ms": snap.get("received_at_ms"),
            "automatic_preparation": preparation,
        }

    def _wait_for_feedback(self, command: dict, stop_event=None) -> dict:
        deadline = time.monotonic() + self._settle_timeout
        joint_name = command["joint_name"]
        target = command["target_position_rad_internal"]
        before = command.get("feedback_before_ms")
        latest = None
        latest_received = None
        while time.monotonic() < deadline:
            if stop_event is not None and stop_event.is_set():
                return {
                    "verified": False,
                    "cancelled": True,
                    "actual_position_deg": None if latest is None else math.degrees(latest),
                    "position_error_deg": None if latest is None else math.degrees(abs(latest - target)),
                    "feedback_received_at_ms": None,
                }
            snap = self._client.snapshot()
            actual = snap.get("joints", {}).get(joint_name)
            received = snap.get("received_at_ms")
            if received is not None:
                latest_received = received
            is_new = received is not None and (before is None or received > before)
            if snap.get("fresh") and actual is not None:
                latest = float(actual)
                error = abs(latest - target)
                if is_new and error <= self._settle_tolerance:
                    return {
                        "verified": True,
                        "actual_position_deg": math.degrees(latest),
                        "position_error_deg": math.degrees(error),
                        "feedback_received_at_ms": received,
                    }
            time.sleep(0.02)
        return {
            "verified": False,
            "actual_position_deg": None if latest is None else math.degrees(latest),
            "position_error_deg": None if latest is None else math.degrees(abs(latest - target)),
            "feedback_received_at_ms": latest_received,
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
        current = command["current_position_rad_internal"]
        target = command["target_position_rad_internal"]
        duration_s = command["duration_s"]
        steps = max(
            int(math.ceil(abs(target - current) / self._max_step)),
            int(math.ceil(duration_s * self._publish_rate)),
            1,
        )
        published = False
        conflict_result = None
        try:
            for index in range(1, steps + 1):
                if stop_event.is_set():
                    break
                conflict_result = self._external_command_conflict(
                    self._router.status())
                if conflict_result:
                    break
                position = current + (target - current) * index / steps
                published = self._publish(joint_name, position) or published
                stop_event.wait(duration_s / steps if duration_s else 0.0)

            if conflict_result:
                result = conflict_result
            elif stop_event.is_set():
                held = self._hold_current(joint_name)
                result = {
                    "ok": True,
                    "state": "stopped",
                    "code": "CANCELLED",
                    "message": "Lower-body adjustment cancelled; latest measured position held",
                    "command": _command_view(command),
                    "feedback_verified": False,
                    "hold_command_published": held,
                }
            elif not published:
                result = _failure(
                    "PUBLISH_FAILED",
                    "Q5 lower-body command could not be published",
                    command=_command_view(command),
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
                        "command": _command_view(command),
                        "feedback_verified": False,
                        "feedback": feedback,
                        "hold_command_published": held,
                    }
                elif feedback["verified"]:
                    result = {
                        "ok": True,
                        "state": "succeeded",
                        "command": _command_view(command),
                        "feedback_verified": True,
                        "feedback": feedback,
                    }
                else:
                    result = _failure(
                        "FEEDBACK_TIMEOUT",
                        "Command was published but fresh target feedback was not verified before timeout",
                        command=_command_view(command),
                        feedback=feedback,
                        settle_timeout_s=self._settle_timeout,
                        settle_tolerance_deg=self._settle_tolerance_deg,
                    )
        except Exception as exc:
            result = _failure(
                "INTERNAL_ERROR",
                "Lower-body motion worker failed",
                exception=str(exc),
                command=_command_view(command),
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
                "command": _command_view(command),
                "no_op": True,
                "feedback_verified": True,
                "message": "Joint is already at the requested absolute target; no command was published",
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
            self._active_command = _command_view(command)
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
            "command": _command_view(command),
            "feedback_verified": False,
        }


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
