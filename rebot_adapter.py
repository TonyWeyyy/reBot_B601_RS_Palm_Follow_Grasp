#!/usr/bin/env python3
"""
reBot Arm B601-RS 机械臂适配层（运动下发 + 夹爪封装 + Pinocchio 运动学/重力动力学）
==================================================================================
将 B601-RS 的独立 7 路 RobStride 底层控制器（``rs_controller.ReBotRSMITController``）
与 Pinocchio 运动学/重力模型封装为与上层 v14/v15 四层流水线 100% 兼容的统一弧度/米制接口：

架构与安全边界：
  1. 单 CAN 主机 7 轴管理：6 个机械臂关节（RS06 × 3 + RS00 × 3）与 J7 夹爪（RS00）共用
     同一个 ``ReBotRSMITController``（默认 ``can0``），严禁额外打开第二个 ``motorbridge`` 实例；
  2. 单位与零偏边界：底层控制器外部接口统一使用【度 / 度每秒】（7 通道）；本适配器对外与
     Pinocchio 一律使用【弧度 / 弧度每秒 / 米 / Nm】（6 关节）；``config/joint_offset.json``
     仅注入 Pinocchio 广义坐标（FK/雅可比/重力），绝不改写下发给电机的目标角；
  3. Pinocchio 双模型线程隔离：主模型 ``self.model / self.data`` 供 100Hz 伺服线程及标定脚本；
     独立重力模型 ``self._grav_model / self._grav_data`` + ``self._grav_lock`` 专供 200Hz MIT
     线程计算静重力前馈，按 ``config/rs_mit.yaml`` 挂载腕部 Gemini 305 + 相机壳 3.0 质点（0.095 kg）；
  4. 安全停机红线：``stop()`` 强制走 ``return_to_zero=True, wait=True``（缓慢回零到位后失能）。
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Optional

import numpy as np

try:
    import pinocchio as pin
except ImportError as exc:
    raise ImportError("请先安装 Pinocchio: pip install pin") from exc

from rs_controller import DEFAULT_RS_MIT_CONFIG_PATH, ReBotRSMITController

_HERE = os.path.dirname(os.path.abspath(__file__))

# 仅使用本工程内置的 B601-RS URDF（100% 独立自包含，无外部仓库路径依赖）
LOCAL_RS_URDF_PATH = os.path.join(_HERE, "urdf", "Rebot_B601_RS", "urdf", "ReBot_Arm_RS.urdf")
URDF_PATH = LOCAL_RS_URDF_PATH

B601_RS_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# 软件关节限位余量（rad）：从 URDF 机械上下限各向内收缩 10°
JOINT_LIMIT_MARGIN = np.deg2rad(10.0)

# 关节零位修正（仅作用于 Pinocchio FK/雅可比/重力，不改写下发给电机的角度）
_JOINT_OFFSET_FILE = os.path.join(_HERE, "config", "joint_offset.json")
if os.path.exists(_JOINT_OFFSET_FILE):
    try:
        with open(_JOINT_OFFSET_FILE, encoding="utf-8") as _f:
            JOINT_OFFSET = np.asarray(json.load(_f)["joint_offset_rad"], dtype=np.float64)
        print(
            f"[RebotAdapter] 已加载关节零位修正 (度): "
            f"{np.rad2deg(JOINT_OFFSET).round(2).tolist()}"
        )
    except Exception:
        JOINT_OFFSET = np.zeros(6, dtype=np.float64)
else:
    JOINT_OFFSET = np.zeros(6, dtype=np.float64)
    print(
        "[RebotAdapter] ⚠️ 未找到 config/joint_offset.json，FK 将不带零位修正。"
        "请先运行 tools/handeye_joint_calib.py"
    )

# 每关节最大速度（rad/s）安全包络：承重抗重力关节 J2/J3 严禁超过 2.0 rad/s
MAX_JOINT_SPEED = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0], dtype=np.float64)
HARD_MAX_JOINT_SPEED = np.array([2.5, 2.0, 2.0, 3.0, 3.0, 2.5], dtype=np.float64)

# 夹爪（J7，RS00）参数
GRIPPER_SPEED_DEG_S = 500.0
GRIPPER_SOFT_LIMIT_DEG = (-100.0, 400.0)

CTRL_DT = 0.02


class _ArmIO:
    """``ReBotRSMITController`` 隔离封装层（单 CAN 主机 ``can0``，7 通道统一使用【度】）。"""

    def __init__(self, rs_config_path: str = DEFAULT_RS_MIT_CONFIG_PATH) -> None:
        self.arm = ReBotRSMITController(config_path=rs_config_path)
        self.ext_cfg = dict(self.arm.ext_cfg)
        self.ext_info = {
            "active": bool(self.ext_cfg.get("enable", True)),
            "phase": self.arm.EXT_PHASE if self.ext_cfg.get("enable", True) else None,
            "reason": "ok" if self.ext_cfg.get("enable", True) else "mit_ext.enable=false",
            "cfg": dict(self.ext_cfg),
        }
        # enable_esc=False 避免抢占主程序 Esc/q 退出路径，保留 SIGINT/SIGTERM 两段式回零信号
        self.arm.start(enable_esc=False)
        self.arm.set_max_speeds(np.rad2deg(MAX_JOINT_SPEED).tolist() + [GRIPPER_SPEED_DEG_S])
        try:
            self.gripper_deg = float(self.arm.read_joint_angles()[6])
        except Exception:
            self.gripper_deg = 0.0

    def send_joint_positions_deg(self, q_deg_7) -> None:
        self.arm.set_joint_angles(q_deg_7)

    def send_joint_mit_targets_deg(self, q_deg_7, dq_deg_s_7) -> None:
        self.arm.set_joint_mit_targets(q_deg_7, dq_deg_s_7)

    def read_joint_positions_deg(self) -> np.ndarray:
        return np.array(self.arm.read_joint_angles()[:6], dtype=np.float64)

    def feedback_snapshot(self):
        try:
            return self.arm.get_feedback_snapshot()
        except Exception:
            return None

    def loop_stats(self):
        try:
            return self.arm.get_loop_stats()
        except Exception:
            return None

    def stop(self) -> None:
        """安全停机：缓慢回零 → 到位保持 → 失能（严禁改为 return_to_zero=False）。"""
        try:
            self.arm.stop(return_to_zero=True, wait=True)
        except Exception:
            pass


class RebotAdapter:
    """reBot Arm B601-RS 高层适配器（对外统一使用弧度 / 米 / Nm，封装 6 关节 + J7 夹爪）。"""

    def __init__(
        self,
        urdf_path: str = URDF_PATH,
        joint_names: list[str] = B601_RS_JOINT_NAMES,
        rs_config_path: str = DEFAULT_RS_MIT_CONFIG_PATH,
    ) -> None:
        self.io = _ArmIO(rs_config_path=rs_config_path)

        urdf_abs = os.path.expanduser(urdf_path)
        self.model = pin.buildModelFromUrdf(urdf_abs)
        self.data = self.model.createData()
        self.joint_names = list(joint_names)
        self.motor_count = 6

        # 独立 Pinocchio 重力模型（线程安全隔离，挂载 0.095 kg Gemini 305 + 相机壳 3.0 质点）
        self._grav_lock = threading.Lock()
        self._grav_model = pin.buildModelFromUrdf(urdf_abs)
        ext_cfg = getattr(self.io, "ext_cfg", {}) or {}
        payload_m = float(ext_cfg.get("gravity_payload_mass_kg", 0.0))
        payload_com = np.asarray(
            ext_cfg.get("gravity_payload_com_link6_m", [0.0, 0.0, 0.0]),
            dtype=np.float64,
        )
        if payload_m > 1e-6:
            jid6 = self._grav_model.getJointId(self.joint_names[-1])
            inertia = pin.Inertia(payload_m, payload_com, np.zeros((3, 3), dtype=np.float64))
            self._grav_model.appendBodyToJoint(jid6, inertia, pin.SE3.Identity())
        self._grav_data = self._grav_model.createData()
        self._grav_idx_q = [
            self._grav_model.joints[self._grav_model.getJointId(n)].idx_q for n in self.joint_names
        ]
        self._grav_idx_v = [
            self._grav_model.joints[self._grav_model.getJointId(n)].idx_v for n in self.joint_names
        ]
        self._grav_q_buf = np.zeros(self._grav_model.nq, dtype=np.float64)

        self.q_min = np.zeros(6, dtype=np.float64)
        self.q_max = np.zeros(6, dtype=np.float64)
        for i, name in enumerate(self.joint_names):
            jid = self.model.getJointId(name)
            qi = self.model.joints[jid].idx_q
            self.q_min[i] = self.model.lowerPositionLimit[qi] + JOINT_LIMIT_MARGIN
            self.q_max[i] = self.model.upperPositionLimit[qi] - JOINT_LIMIT_MARGIN

        q_now = self.get_current_pos()
        self._q_target = q_now.copy()
        self._last_cmd_time = time.time()
        self._max_joint_speed = MAX_JOINT_SPEED.copy()

        self.io.arm.set_gravity_torque_fn(self._gravity_torque_callback)

    def set_joint_speed_limits(self, limits_rad) -> np.ndarray:
        arr = np.asarray(limits_rad, dtype=np.float64)
        if arr.ndim == 0:
            arr = np.full(6, float(arr), dtype=np.float64)
        if arr.shape != (6,) or not np.all(np.isfinite(arr)) or np.any(arr <= 0.0):
            raise ValueError("分关节速度上限必须为 6 维有限正数向量")
        clamped = np.clip(arr, 0.1, HARD_MAX_JOINT_SPEED)
        self._max_joint_speed = clamped.copy()
        self.io.arm.set_max_speeds(np.rad2deg(self._max_joint_speed).tolist() + [GRIPPER_SPEED_DEG_S])
        return self._max_joint_speed.copy()

    # ── 状态读取 ─────────────────────────────────────────────
    def get_current_pos(self) -> np.ndarray:
        return np.deg2rad(self.io.read_joint_positions_deg())

    def get_command_pos(self) -> np.ndarray:
        return np.deg2rad(np.array(self.io.arm.get_command_angles()[:6], dtype=np.float64))

    def mit_ext_info(self) -> dict:
        return dict(getattr(self.io, "ext_info", {"active": False}))

    def get_mit_loop_stats(self):
        return self.io.loop_stats()

    def get_mit_gains(self) -> tuple[np.ndarray, np.ndarray]:
        motors = self.io.arm.config.motors[:6]
        return (
            np.array([float(m.kp) for m in motors], dtype=np.float64),
            np.array([float(m.kd) for m in motors], dtype=np.float64),
        )

    def get_feedback_state(self, max_age_s: Optional[float] = None):
        snap = self.io.feedback_snapshot()
        if snap is None:
            return None
        if max_age_s is None:
            max_age_s = float(self.io.ext_cfg.get("feedback_max_age_s", 0.05))
        q = np.asarray(snap["q"][:6], dtype=np.float64)
        dq = np.asarray(snap["dq"][:6], dtype=np.float64)
        tau = np.asarray(snap["tau"][:6], dtype=np.float64)
        q_cmd = np.asarray(snap["cmd"][:6], dtype=np.float64)
        valid = bool(
            all(snap["valid"][:6])
            and np.all(np.isfinite(q))
            and np.all(np.isfinite(dq))
            and np.all(np.isfinite(tau))
            and 0.0 <= snap["age_s"] <= max_age_s
        )
        return {
            "q": q,
            "dq": dq,
            "tau": tau,
            "q_cmd": q_cmd,
            "t_mos": np.asarray(snap["t_mos"], dtype=np.float64),
            "age_s": float(snap["age_s"]),
            "seq": int(snap["seq"]),
            "valid": valid,
        }

    def get_current_pos_cached(self, max_age_s: Optional[float] = None) -> Optional[np.ndarray]:
        if not self.io.ext_cfg.get("feedback_trusted", False):
            return None
        state = self.get_feedback_state(max_age_s)
        if state is None or not state["valid"]:
            return None
        return state["q"].copy()

    def start_feedback_recording(self) -> None:
        self.io.arm.start_recording()

    def stop_feedback_recording(self) -> dict[str, np.ndarray]:
        rows = self.io.arm.stop_recording()
        if not rows:
            empty = np.zeros((0, 6))
            return {"t": np.zeros(0), "q_cmd": empty, "q": empty, "dq": empty, "tau": empty}
        t = np.array([r[0] for r in rows], dtype=np.float64)
        return {
            "t": t - t[0],
            "q_cmd": np.array([r[1][:6] for r in rows], dtype=np.float64),
            "q": np.array([r[2][:6] for r in rows], dtype=np.float64),
            "dq": np.array([r[3][:6] for r in rows], dtype=np.float64),
            "tau": np.array([r[4][:6] for r in rows], dtype=np.float64),
        }

    # ── 标定/辨识专用目标接口 ────────────────────────────────
    def set_joint_targets_direct(self, q_rad, dq_rad=None) -> np.ndarray:
        q = np.asarray(q_rad, dtype=np.float64)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError("关节目标必须为 6 维有限向量")
        q_clipped = np.clip(q, self.q_min, self.q_max)
        self._q_target = q_clipped
        self._last_cmd_time = time.time()
        q_deg = np.rad2deg(self._q_target).tolist() + [self._gripper_keep_deg()]
        if dq_rad is None:
            self.io.send_joint_positions_deg(q_deg)
        else:
            dq = np.asarray(dq_rad, dtype=np.float64)
            if dq.shape != (6,) or not np.all(np.isfinite(dq)):
                raise ValueError("前馈速度必须为 6 维有限向量")
            dq = np.clip(dq, -HARD_MAX_JOINT_SPEED, HARD_MAX_JOINT_SPEED)
            hit_lo = (q <= self.q_min) & (dq < 0.0)
            hit_hi = (q >= self.q_max) & (dq > 0.0)
            dq = np.where(hit_lo | hit_hi, 0.0, dq)
            dq_deg_s = np.rad2deg(dq).tolist() + [0.0]
            self.io.send_joint_mit_targets_deg(q_deg, dq_deg_s)
        return self._q_target.copy()

    def override_joint_speed_limits(self, limits_rad) -> np.ndarray:
        arr = np.asarray(limits_rad, dtype=np.float64)
        if arr.shape != (6,) or not np.all(np.isfinite(arr)) or np.any(arr <= 0.0):
            raise ValueError("限速必须为 6 维有限正数向量")
        clamped = np.clip(arr, 0.01, HARD_MAX_JOINT_SPEED)
        self.io.arm.set_max_speeds(np.rad2deg(clamped).tolist() + [GRIPPER_SPEED_DEG_S])
        return clamped

    def restore_joint_speed_limits(self) -> None:
        self.io.arm.set_max_speeds(np.rad2deg(self._max_joint_speed).tolist() + [GRIPPER_SPEED_DEG_S])

    # ── 运动指令 ─────────────────────────────────────────────
    def joint_vel_cmd(self, dq_rad) -> None:
        now = time.time()
        dt = min(max(now - self._last_cmd_time, 1e-3), 3 * CTRL_DT)
        self._last_cmd_time = now

        dq_send = np.zeros(6, dtype=np.float64)
        if dq_rad is not None:
            dq = np.asarray(dq_rad, dtype=np.float64)
            lim = self._max_joint_speed
            ratios = np.abs(dq) / lim
            max_ratio = float(np.max(ratios)) if dq.size else 0.0
            if max_ratio > 1.0:
                dq = dq / max_ratio
            dq = np.clip(dq, -lim, lim)
            q_next = self._q_target + dq * dt
            hit_lo = (q_next <= self.q_min) & (dq < 0.0)
            hit_hi = (q_next >= self.q_max) & (dq > 0.0)
            dq_send = np.where(hit_lo | hit_hi, 0.0, dq)
            self._q_target = np.clip(q_next, self.q_min, self.q_max)

        q_deg = np.rad2deg(self._q_target).tolist() + [self._gripper_keep_deg()]
        dq_deg_s = np.rad2deg(dq_send).tolist() + [0.0]
        self.io.send_joint_mit_targets_deg(q_deg, dq_deg_s)

    def hold(self) -> None:
        self._q_target = self.get_command_pos().copy()
        self._last_cmd_time = time.time()
        q_deg = np.rad2deg(self._q_target).tolist() + [self._gripper_keep_deg()]
        self.io.send_joint_positions_deg(q_deg)

    def move_to_joints(self, q_rad, speed_deg_s: float = 20.0, wait: bool = True, timeout: float = 15.0) -> None:
        self._q_target = np.clip(np.asarray(q_rad, dtype=np.float64), self.q_min, self.q_max)
        self.io.arm.set_max_speeds([speed_deg_s] * 6 + [GRIPPER_SPEED_DEG_S])
        q_deg = np.rad2deg(self._q_target).tolist() + [self._gripper_keep_deg()]
        self.io.send_joint_positions_deg(q_deg)
        if wait:
            t0 = time.time()
            while time.time() - t0 < timeout:
                if np.max(np.abs(self.get_command_pos() - self._q_target)) < np.deg2rad(1.0):
                    q_act = self.get_current_pos_cached()
                    if q_act is None:
                        q_act = self.get_current_pos()
                    if np.max(np.abs(q_act - self._q_target)) < np.deg2rad(2.0):
                        break
                time.sleep(0.1)
        self.io.arm.set_max_speeds(np.rad2deg(self._max_joint_speed).tolist() + [GRIPPER_SPEED_DEG_S])
        self._last_cmd_time = time.time()

    def stop(self) -> None:
        self.io.stop()

    def _gripper_keep_deg(self) -> float:
        return self.io.gripper_deg

    # ── 夹爪（J7）控制 ───────────────────────────────────────
    def set_gripper_deg(self, deg: float) -> None:
        lo, hi = GRIPPER_SOFT_LIMIT_DEG
        self.io.gripper_deg = float(np.clip(deg, lo, hi))

    def get_gripper_deg(self) -> float:
        return float(self.io.gripper_deg)

    # ── Pinocchio 运动学与重力模型 ───────────────────────────
    def _build_q_full(self, joint_positions) -> np.ndarray:
        q_full = np.zeros(self.model.nq, dtype=np.float64)
        for i, name in enumerate(self.joint_names):
            jid = self.model.getJointId(name)
            q_full[self.model.joints[jid].idx_q] = joint_positions[i] + JOINT_OFFSET[i]
        return q_full

    def forward_kinematics(self) -> dict[str, np.ndarray]:
        q = self._build_q_full(self.get_current_pos())
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        jid = self.model.getJointId(self.joint_names[-1])
        oMi = self.data.oMi[jid]
        return {"position": oMi.translation.copy(), "rotation": oMi.rotation.copy()}

    def gravity_torque(self, q_rad, include_payload: bool = False) -> np.ndarray:
        if include_payload:
            return np.asarray(self._gravity_torque_callback(q_rad), dtype=np.float64)
        q_full = self._build_q_full(q_rad)
        g_full = pin.computeGeneralizedGravity(self.model, self.data, q_full)
        cols = [self.model.joints[self.model.getJointId(n)].idx_v for n in self.joint_names]
        return g_full[cols]

    def _gravity_torque_callback(self, q_rad) -> list[float]:
        with self._grav_lock:
            self._grav_q_buf[:] = 0.0
            for i in range(6):
                self._grav_q_buf[self._grav_idx_q[i]] = float(q_rad[i]) + float(JOINT_OFFSET[i])
            g_full = pin.computeGeneralizedGravity(
                self._grav_model, self._grav_data, self._grav_q_buf
            )
            return [float(g_full[idx]) for idx in self._grav_idx_v]

    def set_reach_radius(self, reach_r_m: float) -> None:
        self.io.arm.set_reach_radius(reach_r_m)

    def get_safety_status(self):
        try:
            return self.io.arm.get_safety_status()
        except Exception:
            return None

    def clear_safety_fault(self) -> None:
        self.io.arm.clear_safety_fault()
