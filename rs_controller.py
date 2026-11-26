#!/usr/bin/env python3
"""
reBot Arm B601-RS 底层实时控制器（RobStride RS06 + RS00 / motorbridge）
=======================================================================
专为 Seeed reBot Arm B601-RS 设计的独立 200Hz 实时控制引擎，直接封装 ``motorbridge``
（SocketCAN ``can0``，``host_id=0xFD``，J1~J3 为 ``rs-06``，J4~J6 与 J7 夹爪为 ``rs-00``），
零依赖外部 ``rebot_control`` 源码仓库，并完整内置经实机验收定稿的 P0~P4 全部底层能力：

1. **P0 · S1 CAN 瞬时故障容错（``CanFaultMonitor``）与 200Hz 零额外请求反馈缓存**：
   - 单帧/偶发 CAN 发送异常只计数并限频告警，仅连续失败持续 ``>= can_fault_window_s``
     （默认 30ms @ 200Hz → 6 拍）才判通信故障，默认走「缓慢安全回零 → 失能」，杜绝半空坠臂；
   - 同拍在 ``_io_lock_guard()`` 内零额外 CAN 请求读取 ``motor.get_state()`` 应答缓存
     （pos/vel/torq/t_mos/t_rotor/status），支持 ``feedback_trusted`` 替代阻塞式 ``0x7019``。
2. **P1 · MIT 速度前馈 ``dq_des`` + 200Hz FOH 一阶保持插值 + 40ms 看门狗**（``compute_p1_mit_step``）：
   - ``set_joint_mit_targets(angles_deg, velocities_deg_s)`` 与老接口 ``set_joint_angles()`` 完全兼容；
   - FOH 向后插值消除 100Hz→200Hz 阶梯纹波且绝不越过目标/软限位；
   - 硬隔离：J7 夹爪（索引 6）以及 ``set_joint_angles()`` / ``hold()`` / 回零路径恒强制 ``dq_des = 0``。
3. **P3 · 静态重力前馈 ``tau_ff``（``compute_p3_gravity_tau``）与伸展自适应阻尼（``compute_p3_gains``）**：
   - 由 ``RebotAdapter`` 注入线程隔离 Pinocchio 重力回调（含 0.095 kg Gemini 305 + 相机壳 3.0 载荷）；
   - 0.5s 淡入、30 Nm/s 斜率限幅、双重硬封顶、J1/J7 恒为 0、回零近零位支架平滑淡出（``cradle_zero_fade``）；
   - 基于水平伸展半径 ``reach_r`` 平滑调度 J1~J3 阻尼 ``kd``，抑制悬臂长伸展共振。
4. **P4 · M6 200Hz 主动安全监督（``SafetySupervisor``）与三级过温保护**：
   - 持续监测 6 轴跟踪误差（>12° 持续 ≥80ms）与 RS06/RS00 力矩上限，触发后原地软保持（维持重力
     补偿，绝不空中失能），排查确认安全后调用 ``clear_safety_fault()`` 恢复；
   - 90~110°C 线性热降额 + 2Hz 三级过温保护（110°C 报警 / 125°C 高温缓慢回零 / 140°C 紧急断开）。
5. **drop-proof 安全停机与信号处理**：
   - ``stop(return_to_zero=True, wait=True)``、Esc、第 1 次 Ctrl+C 均走 smoothstep 缓慢回零
     （含重力淡出与 CAN 发送瞬时重试）→ 到达零位保持 → 失能释放总线；回零途中第 2 次 Ctrl+C 立即中止失能。
"""

from __future__ import annotations

import math
import os
import signal
import threading
import time
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any, Callable, Iterator, Optional, Sequence

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

try:
    from motorbridge import Controller, Mode
except ImportError:  # pragma: no cover - 允许无 motorbridge 的纯单测环境导入纯算法
    Controller = None  # type: ignore[assignment]
    Mode = None  # type: ignore[assignment]

try:
    from pynput import keyboard as _pynput_keyboard
except Exception:  # pragma: no cover
    _pynput_keyboard = None


DEFAULT_RS_MIT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "config", "rs_mit.yaml"
)

# 代码级绝对力矩硬天花板（Nm），即使改坏 YAML 也不得突破
HARD_MAX_GRAVITY_TAU_NM = (0.0, 14.0, 14.0, 3.5, 2.5, 1.5)
# 电机峰值力矩硬件上限（RS06=36 Nm, RS00=14 Nm）
HARD_MOTOR_PEAK_TAU_NM = (36.0, 36.0, 36.0, 14.0, 14.0, 14.0)

_CAN_FAULT_ACTIONS = ("return_to_zero", "emergency")


# ─────────────────────────────────────────────────────────────
# 硬件与 MIT 扩展配置数据结构与校验
# ─────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RSMotorConfig:
    """单台 RobStride 电机配置。"""

    name: str
    motor_id: int
    model: str
    kp: float
    kd: float


@dataclass(frozen=True)
class TemperatureThresholds:
    """三级过温保护阈值（°C），满足 alarm_c < return_zero_c < disconnect_c。"""

    alarm_c: float = 110.0
    return_zero_c: float = 125.0
    disconnect_c: float = 140.0


@dataclass(frozen=True)
class ReturnZeroConfig:
    """安全停机 smoothstep 缓慢回零参数。"""

    max_speed_deg_s: float = 30.0
    thermal_max_speed_deg_s: float = 30.0
    min_time_s: float = 3.0
    settle_time_s: float = 0.30


@dataclass(frozen=True)
class RSHardwareConfig:
    """B601-RS 硬件与 CAN 总线配置。"""

    name: str = "reBot_B601_RS"
    channel: str = "can0"
    host_id: int = 0xFD
    control_hz: float = 200.0
    telemetry_hz: float = 2.0
    return_zero: ReturnZeroConfig = field(default_factory=ReturnZeroConfig)
    temperatures: TemperatureThresholds = field(default_factory=TemperatureThresholds)
    motors: tuple[RSMotorConfig, ...] = (
        RSMotorConfig("joint1", 1, "rs-06", 72.0, 3.6),
        RSMotorConfig("joint2", 2, "rs-06", 216.0, 12.0),
        RSMotorConfig("joint3", 3, "rs-06", 216.0, 12.0),
        RSMotorConfig("joint4", 4, "rs-00", 50.0, 5.0),
        RSMotorConfig("joint5", 5, "rs-00", 50.0, 4.0),
        RSMotorConfig("joint6", 6, "rs-00", 50.0, 4.0),
        RSMotorConfig("gripper", 7, "rs-00", 50.0, 4.0),
    )


DEFAULT_RS_MIT_EXT_CFG: dict[str, Any] = {
    "enable": True,
    # P0
    "can_fault_window_s": 0.03,
    "can_fault_action": "return_to_zero",
    "can_error_log_interval_s": 1.0,
    "feedback_cache": True,
    "feedback_poll_once": False,
    "feedback_trusted": True,
    "feedback_max_age_s": 0.05,
    "record_max_s": 30.0,
    # P1
    "vel_ff_enable": True,
    "vel_ff_scale": [0.70, 0.85, 0.85, 0.90, 0.90, 0.90],
    "foh_interp_enable": True,
    "foh_upper_dt_s": 0.010,
    "vel_ff_watchdog_s": 0.040,
    "vel_ff_watchdog_tau_s": 0.030,
    "vel_ff_lpf_tau_s": 0.008,
    # P3 M2
    "gravity_ff_enable": True,
    "gravity_ff_scale": [0.0, 0.65, 0.80, 0.80, 0.0, 0.0],
    "gravity_ff_max_nm": [0.0, 12.0, 12.0, 2.5, 2.0, 1.0],
    "gravity_payload_mass_kg": 0.095,
    "gravity_payload_com_link6_m": [0.055, 0.000, 0.075],
    "gravity_ff_ramp_in_s": 0.50,
    "gravity_ff_slew_nm_s": 30.0,
    # P3 M3
    "gain_sched_enable": True,
    "gain_sched_r_min_m": 0.18,
    "gain_sched_r_max_m": 0.36,
    "gain_sched_kd_near": [4.6, 13.5, 14.0, 5.0, 4.0, 4.0],
    "gain_sched_kd_far": [6.4, 14.5, 15.0, 5.0, 4.0, 4.0],
    "gain_sched_kp_scale": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    "gain_sched_slew_per_s": 10.0,
    # P4 M6
    "supervisor_enable": True,
    "track_err_limit_deg": 12.0,
    "track_err_window_s": 0.08,
    "tau_limit_nm": [18.0, 28.0, 28.0, 10.0, 10.0, 10.0],
    "tau_limit_window_s": 0.08,
    "thermal_derate_start_c": 90.0,
    "thermal_derate_full_c": 110.0,
    "thermal_derate_min_scale": 0.40,
}


def validate_rs_mit_ext_cfg(cfg: Optional[dict[str, Any]]) -> dict[str, Any]:
    """校验并补全 B601-RS MIT 扩展配置字典（缺省项自动填充 ``DEFAULT_RS_MIT_EXT_CFG``）。"""
    if cfg is None:
        cfg = {}
    if not isinstance(cfg, dict):
        raise ValueError("mit_ext 配置必须是字典")
    unknown = set(cfg) - set(DEFAULT_RS_MIT_EXT_CFG)
    if unknown:
        raise ValueError(f"mit_ext 存在未知配置项: {sorted(unknown)}")
    merged = dict(DEFAULT_RS_MIT_EXT_CFG)
    merged.update(cfg)

    for key in (
        "enable",
        "feedback_cache",
        "feedback_poll_once",
        "feedback_trusted",
        "vel_ff_enable",
        "foh_interp_enable",
        "gravity_ff_enable",
        "gain_sched_enable",
        "supervisor_enable",
    ):
        if not isinstance(merged[key], bool):
            raise ValueError(f"mit_ext.{key} 必须是 true/false")

    def _bounded(key: str, lo: float, hi: float) -> None:
        value = merged[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"mit_ext.{key} 必须是数值")
        fv = float(value)
        if not math.isfinite(fv) or not lo <= fv <= hi:
            raise ValueError(f"mit_ext.{key}={fv} 超出安全范围 [{lo}, {hi}]")
        merged[key] = fv

    def _vec(key: str, length: int, lo: float | Sequence[float], hi: float | Sequence[float]) -> None:
        raw = merged[key]
        if not isinstance(raw, (list, tuple)) or len(raw) != length:
            raise ValueError(f"mit_ext.{key} 必须是 {length} 维列表")
        out: list[float] = []
        for idx, val in enumerate(raw):
            if isinstance(val, bool) or not isinstance(val, (int, float)):
                raise ValueError(f"mit_ext.{key}[{idx}] 必须是数值")
            fv = float(val)
            lo_i = float(lo[idx]) if isinstance(lo, (list, tuple)) else float(lo)
            hi_i = float(hi[idx]) if isinstance(hi, (list, tuple)) else float(hi)
            if not math.isfinite(fv) or not lo_i <= fv <= hi_i:
                raise ValueError(f"mit_ext.{key}[{idx}]={fv} 超出安全范围 [{lo_i}, {hi_i}]")
            out.append(fv)
        merged[key] = out

    _bounded("can_fault_window_s", 0.01, 0.10)
    _bounded("can_error_log_interval_s", 0.1, 10.0)
    _bounded("feedback_max_age_s", 0.01, 0.20)
    _bounded("record_max_s", 1.0, 120.0)
    _bounded("foh_upper_dt_s", 0.005, 0.050)
    _bounded("vel_ff_watchdog_s", 0.015, 0.150)
    _bounded("vel_ff_watchdog_tau_s", 0.005, 0.100)
    _bounded("vel_ff_lpf_tau_s", 0.0, 0.050)
    _vec("vel_ff_scale", 6, 0.0, 1.0)

    _vec("gravity_ff_scale", 6, 0.0, [0.0, 0.95, 0.95, 0.95, 0.95, 0.95])
    _vec("gravity_ff_max_nm", 6, 0.0, HARD_MAX_GRAVITY_TAU_NM)
    _bounded("gravity_payload_mass_kg", 0.0, 1.0)
    _vec("gravity_payload_com_link6_m", 3, -0.30, 0.30)
    _bounded("gravity_ff_ramp_in_s", 0.05, 3.0)
    _bounded("gravity_ff_slew_nm_s", 2.0, 80.0)

    _bounded("gain_sched_r_min_m", 0.05, 0.35)
    _bounded("gain_sched_r_max_m", 0.15, 0.45)
    if merged["gain_sched_r_max_m"] <= merged["gain_sched_r_min_m"]:
        raise ValueError("mit_ext.gain_sched_r_max_m 必须大于 gain_sched_r_min_m")
    _vec("gain_sched_kd_near", 6, 0.5, [8.0, 18.0, 18.0, 5.0, 5.0, 5.0])
    _vec("gain_sched_kd_far", 6, 0.5, [8.0, 18.0, 18.0, 5.0, 5.0, 5.0])
    _vec("gain_sched_kp_scale", 6, 0.8, 1.5)
    _bounded("gain_sched_slew_per_s", 1.0, 40.0)

    _bounded("track_err_limit_deg", 4.0, 25.0)
    _bounded("track_err_window_s", 0.02, 0.50)
    _vec("tau_limit_nm", 6, 2.0, HARD_MOTOR_PEAK_TAU_NM)
    _bounded("tau_limit_window_s", 0.02, 0.50)
    _bounded("thermal_derate_start_c", 50.0, 105.0)
    _bounded("thermal_derate_full_c", 60.0, 120.0)
    if merged["thermal_derate_full_c"] <= merged["thermal_derate_start_c"]:
        raise ValueError("mit_ext.thermal_derate_full_c 必须大于 thermal_derate_start_c")
    _bounded("thermal_derate_min_scale", 0.20, 1.0)

    if merged["can_fault_action"] not in _CAN_FAULT_ACTIONS:
        raise ValueError(f"mit_ext.can_fault_action 必须是 {_CAN_FAULT_ACTIONS} 之一")
    if merged["feedback_trusted"] and not merged["feedback_cache"]:
        raise ValueError("feedback_trusted=true 要求 feedback_cache=true")
    return merged


def _parse_hardware_cfg(raw: Optional[dict[str, Any]]) -> RSHardwareConfig:
    """解析并校验 `hardware` 配置段。"""
    default_hw = RSHardwareConfig()
    if not raw:
        return default_hw
    if not isinstance(raw, dict):
        raise ValueError("hardware 配置必须是字典")

    channel = str(raw.get("channel", default_hw.channel)).strip()
    if not channel:
        raise ValueError("hardware.channel 不能为空")
    host_id = int(raw.get("host_id", default_hw.host_id))
    control_hz = float(raw.get("control_hz", default_hw.control_hz))
    telemetry_hz = float(raw.get("telemetry_hz", default_hw.telemetry_hz))
    if not math.isfinite(control_hz) or not 20.0 <= control_hz <= 500.0:
        raise ValueError(f"hardware.control_hz={control_hz} 越界 [20, 500]")
    if not math.isfinite(telemetry_hz) or not 0.2 <= telemetry_hz <= 20.0:
        raise ValueError(f"hardware.telemetry_hz={telemetry_hz} 越界 [0.2, 20]")

    rz_raw = raw.get("return_zero", {}) or {}
    rz = ReturnZeroConfig(
        max_speed_deg_s=float(rz_raw.get("max_speed_deg_s", default_hw.return_zero.max_speed_deg_s)),
        thermal_max_speed_deg_s=float(
            rz_raw.get("thermal_max_speed_deg_s", default_hw.return_zero.thermal_max_speed_deg_s)
        ),
        min_time_s=float(rz_raw.get("min_time_s", default_hw.return_zero.min_time_s)),
        settle_time_s=float(rz_raw.get("settle_time_s", default_hw.return_zero.settle_time_s)),
    )
    if rz.max_speed_deg_s <= 0 or rz.thermal_max_speed_deg_s <= 0 or rz.min_time_s <= 0:
        raise ValueError("hardware.return_zero 速度与最短时长必须为正数")

    t_raw = raw.get("temperatures", {}) or {}
    temps = TemperatureThresholds(
        alarm_c=float(t_raw.get("alarm_c", default_hw.temperatures.alarm_c)),
        return_zero_c=float(t_raw.get("return_zero_c", default_hw.temperatures.return_zero_c)),
        disconnect_c=float(t_raw.get("disconnect_c", default_hw.temperatures.disconnect_c)),
    )
    if not (temps.alarm_c < temps.return_zero_c < temps.disconnect_c):
        raise ValueError("hardware.temperatures 必须满足 alarm_c < return_zero_c < disconnect_c")

    motors_raw = raw.get("motors")
    if motors_raw is None:
        motors = default_hw.motors
    else:
        if not isinstance(motors_raw, (list, tuple)) or len(motors_raw) != 7:
            raise ValueError("hardware.motors 必须包含 7 路电机（J1~J6 + J7 夹爪）")
        parsed_motors: list[RSMotorConfig] = []
        for idx, item in enumerate(motors_raw):
            if not isinstance(item, dict):
                raise ValueError(f"hardware.motors[{idx}] 必须是字典")
            m = RSMotorConfig(
                name=str(item.get("name", f"joint{idx + 1}")),
                motor_id=int(item["motor_id"]),
                model=str(item["model"]).strip(),
                kp=float(item["kp"]),
                kd=float(item["kd"]),
            )
            if m.kp < 0.0 or m.kd < 0.0:
                raise ValueError(f"hardware.motors[{idx}] kp/kd 必须非负")
            parsed_motors.append(m)
        motors = tuple(parsed_motors)

    return RSHardwareConfig(
        name=str(raw.get("name", default_hw.name)),
        channel=channel,
        host_id=host_id,
        control_hz=control_hz,
        telemetry_hz=telemetry_hz,
        return_zero=rz,
        temperatures=temps,
        motors=motors,
    )


def load_rs_mit_config(path: Optional[str] = None) -> tuple[RSHardwareConfig, dict[str, Any]]:
    """从 `config/rs_mit.yaml` 加载并校验硬件配置与 MIT 扩展配置。"""
    cfg_path = path or DEFAULT_RS_MIT_CONFIG_PATH
    hw_raw: dict[str, Any] = {}
    ext_raw: dict[str, Any] = {}
    if os.path.exists(cfg_path):
        if yaml is None:
            raise ImportError("读取 rs_mit.yaml 需要 pyyaml")
        with open(cfg_path, encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict):
            raise ValueError(f"{cfg_path} 顶层必须是字典")
        hw_raw = data.get("hardware", {}) or {}
        ext_raw = data.get("mit_ext", {}) or {}
    return _parse_hardware_cfg(hw_raw), validate_rs_mit_ext_cfg(ext_raw)


# ─────────────────────────────────────────────────────────────
# 纯逻辑组件（无硬件依赖，可独立离线单元测试）
# ─────────────────────────────────────────────────────────────
class CanFaultMonitor:
    """S1 CAN 连续发送失败监测器（阈值由 control_hz 推导，禁止硬编码 tick 数）。"""

    def __init__(self, window_s: float, control_hz: float) -> None:
        rate = float(control_hz)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("control_hz 必须为正数")
        self.threshold = max(2, int(round(float(window_s) * rate)))
        self.consecutive = 0
        self.max_consecutive = 0
        self.total_failures = 0
        self.total_success = 0

    def record_success(self) -> None:
        self.consecutive = 0
        self.total_success += 1

    def record_failure(self) -> bool:
        self.consecutive += 1
        self.total_failures += 1
        self.max_consecutive = max(self.max_consecutive, self.consecutive)
        return self.consecutive >= self.threshold


def _percentile(values: Sequence[float], q: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(q * (len(ordered) - 1)))))
    return ordered[index]


def compute_p1_mit_step(
    target: float,
    command: float,
    dq_target: float,
    dq_prev_send: float,
    max_speed: float,
    period: float,
    elapsed: float,
    *,
    vel_ff_enable: bool,
    vel_ff_scale: float,
    foh_enable: bool,
    foh_upper_dt_s: float,
    watchdog_s: float,
    watchdog_tau_s: float,
    lpf_tau_s: float,
    mit_vel_mode: bool,
) -> tuple[float, float, bool]:
    """单关节 200Hz MIT 位置步进与速度前馈 `dq_des` 计算（纯函数）。"""
    max_step = max_speed * period
    if not mit_vel_mode or not vel_ff_enable:
        error = target - command
        step = max(-max_step, min(max_step, error))
        return command + step, 0.0, False

    elapsed_s = max(0.0, float(elapsed))
    if elapsed_s <= watchdog_s:
        wd = 1.0
        wd_active = False
    else:
        wd = math.exp(-(elapsed_s - watchdog_s) / max(watchdog_tau_s, 1e-3))
        if wd < 1e-3:
            wd = 0.0
        wd_active = True

    dq_clamped = max(-max_speed, min(max_speed, float(dq_target))) * wd
    if foh_enable:
        rem_s = max(0.0, foh_upper_dt_s - elapsed_s)
        q_interp = target - dq_clamped * rem_s
    else:
        q_interp = target

    step = max(-max_step, min(max_step, q_interp - command))
    new_cmd = command + step

    error_full = target - command
    if abs(error_full) > 0.0035 and dq_clamped * error_full < 0.0:
        dq_raw = 0.0
    elif abs(dq_clamped) < 1e-4:
        dq_raw = 0.0
    elif abs(error_full) < 1e-4 and elapsed_s > 1.5 * foh_upper_dt_s:
        dq_raw = 0.0
    else:
        dq_raw = dq_clamped * float(vel_ff_scale)

    if lpf_tau_s > 0.0:
        alpha = period / (lpf_tau_s + period)
        new_dq_send = float(dq_prev_send) + alpha * (dq_raw - float(dq_prev_send))
    else:
        new_dq_send = dq_raw

    if dq_raw == 0.0 and abs(new_dq_send) < 1e-3:
        new_dq_send = 0.0
    new_dq_send = max(-max_speed, min(max_speed, new_dq_send))
    return new_cmd, new_dq_send, wd_active


def _smoothstep01(x: float) -> float:
    u = max(0.0, min(1.0, float(x)))
    return u * u * (3.0 - 2.0 * u)


def cradle_zero_fade(q_rad_6: Sequence[float]) -> float:
    """零位机械支架重力补偿淡出因子 `[0.0, 1.0]`（<=0.03 rad 归零，>=0.13 rad 全额）。"""
    lift = max(abs(float(q_rad_6[1])), abs(float(q_rad_6[2])))
    return _smoothstep01((lift - 0.03) / 0.10)


def compute_p3_gravity_tau(
    q_eval_6: Sequence[float],
    prev_tau_6: Sequence[float],
    gravity_fn: Optional[Callable[[Sequence[float]], Sequence[float]]],
    period: float,
    *,
    enabled: bool,
    scales: Sequence[float],
    max_nm: Sequence[float],
    ramp_factor: float,
    slew_nm_s: float,
) -> tuple[list[float], bool]:
    """计算 6 轴静态重力补偿前馈力矩 `tau_ff`（含淡入、零位淡出、双重硬封顶与斜率限速）。"""
    target = [0.0] * 6
    error_flag = False
    zfade = cradle_zero_fade(q_eval_6) if len(q_eval_6) >= 6 else 0.0
    eff_ramp = max(0.0, min(1.0, float(ramp_factor))) * zfade
    if enabled and gravity_fn is not None and eff_ramp > 1e-4:
        try:
            raw = gravity_fn(q_eval_6)
            if len(raw) != 6 or not all(math.isfinite(float(v)) for v in raw):
                error_flag = True
            else:
                for i in range(6):
                    if i == 0:
                        target[0] = 0.0
                        continue
                    cap = min(float(max_nm[i]), float(HARD_MAX_GRAVITY_TAU_NM[i]))
                    val = float(raw[i]) * float(scales[i]) * eff_ramp
                    target[i] = max(-cap, min(cap, val))
        except Exception:  # noqa: BLE001
            error_flag = True

    max_step = max(1e-3, float(slew_nm_s) * float(period))
    out = [0.0] * 6
    for i in range(6):
        prev = float(prev_tau_6[i])
        delta = max(-max_step, min(max_step, target[i] - prev))
        v = prev + delta
        if target[i] == 0.0 and abs(v) < 1e-3:
            v = 0.0
        out[i] = 0.0 if i == 0 else v
    return out, error_flag


def compute_p3_gains(
    base_kp_6: Sequence[float],
    base_kd_6: Sequence[float],
    prev_kp_6: Sequence[float],
    prev_kd_6: Sequence[float],
    reach_r: float,
    period: float,
    *,
    enabled: bool,
    r_min: float,
    r_max: float,
    kd_near: Sequence[float],
    kd_far: Sequence[float],
    kp_scale: Sequence[float],
    slew_per_s: float,
) -> tuple[list[float], list[float]]:
    """计算 6 轴水平伸展度自适应 MIT 增益 `(kp, kd)`（RS06 上限 18.0，RS00 上限 5.0）。"""
    s = _smoothstep01((float(reach_r) - float(r_min)) / max(float(r_max) - float(r_min), 1e-3))
    max_step = max(1e-3, float(slew_per_s) * float(period))
    kp_out = [0.0] * 6
    kd_out = [0.0] * 6
    for i in range(6):
        bkp = float(base_kp_6[i])
        bkd = float(base_kd_6[i])
        if enabled:
            kp_lo, kp_hi = 0.8 * bkp, 1.5 * bkp
            kd_hi_hw = 5.0 if i >= 3 else 18.0
            kd_lo, kd_hi = 0.8 * bkd, min(1.8 * bkd, kd_hi_hw)
            t_kp = max(kp_lo, min(kp_hi, bkp * float(kp_scale[i])))
            raw_kd = float(kd_near[i]) + (float(kd_far[i]) - float(kd_near[i])) * s
            t_kd = max(kd_lo, min(kd_hi, raw_kd))
        else:
            t_kp, t_kd = bkp, bkd
        pkp = float(prev_kp_6[i])
        pkd = float(prev_kd_6[i])
        kp_out[i] = pkp + max(-max_step, min(max_step, t_kp - pkp))
        kd_out[i] = pkd + max(-max_step, min(max_step, t_kd - pkd))
    return kp_out, kd_out


class SafetySupervisor:
    """P4 M6 200Hz 主动安全监督器（碰撞/堵转软保持 + 90~110°C 线性热降额）。"""

    def __init__(self, cfg: dict[str, Any], control_hz: float) -> None:
        rate = float(control_hz)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("control_hz 必须为正数")
        self.enabled = bool(cfg["supervisor_enable"])
        self.track_err_limit_deg = float(cfg["track_err_limit_deg"])
        self.track_thresh_ticks = max(2, int(round(float(cfg["track_err_window_s"]) * rate)))
        self.track_window_ms = float(cfg["track_err_window_s"]) * 1000.0
        self.tau_limit_nm = [float(v) for v in cfg["tau_limit_nm"]]
        self.tau_thresh_ticks = max(2, int(round(float(cfg["tau_limit_window_s"]) * rate)))
        self.tau_window_ms = float(cfg["tau_limit_window_s"]) * 1000.0
        self.thermal_start_c = float(cfg["thermal_derate_start_c"])
        self.thermal_full_c = float(cfg["thermal_derate_full_c"])
        self.thermal_min_scale = float(cfg["thermal_derate_min_scale"])
        self._track_over = 0
        self._tau_over = 0
        self.fault = False
        self.fault_reason = ""
        self.thermal_scale = 1.0
        self.last_track_err_deg = 0.0
        self.last_tau_ratio = 0.0
        self.last_tmos_max = math.nan

    def clear_fault(self) -> None:
        self.fault = False
        self.fault_reason = ""
        self._track_over = 0
        self._tau_over = 0

    def update(
        self,
        q_cmd_6: Sequence[float],
        q_act_6: Sequence[float],
        tau_act_6: Sequence[float],
        t_mos_all: Sequence[Optional[float]],
        valid_6: Sequence[bool],
    ) -> bool:
        finite_temps = [float(t) for t in t_mos_all if t is not None and math.isfinite(float(t))]
        if finite_temps:
            tmax = max(finite_temps)
            self.last_tmos_max = tmax
            if tmax <= self.thermal_start_c:
                self.thermal_scale = 1.0
            elif tmax >= self.thermal_full_c:
                self.thermal_scale = self.thermal_min_scale
            else:
                frac = (tmax - self.thermal_start_c) / max(self.thermal_full_c - self.thermal_start_c, 1e-3)
                self.thermal_scale = 1.0 - frac * (1.0 - self.thermal_min_scale)

        if not self.enabled or not all(valid_6[:6]):
            self._track_over = 0
            self._tau_over = 0
            return False

        worst_err = 0.0
        worst_err_j = 1
        for i in range(6):
            err = abs(math.degrees(float(q_cmd_6[i]) - float(q_act_6[i])))
            if math.isfinite(err) and err > worst_err:
                worst_err = err
                worst_err_j = i + 1
        self.last_track_err_deg = worst_err
        if worst_err > self.track_err_limit_deg:
            self._track_over += 1
        else:
            self._track_over = 0

        worst_ratio = 0.0
        worst_tau = 0.0
        worst_tau_lim = 1.0
        worst_tau_j = 1
        for i in range(6):
            tv = abs(float(tau_act_6[i]))
            lim = self.tau_limit_nm[i]
            ratio = tv / max(lim, 1e-6)
            if math.isfinite(ratio) and ratio > worst_ratio:
                worst_ratio = ratio
                worst_tau = tv
                worst_tau_lim = lim
                worst_tau_j = i + 1
        self.last_tau_ratio = worst_ratio
        if worst_ratio > 1.0:
            self._tau_over += 1
        else:
            self._tau_over = 0

        if self.fault:
            return False

        if self._track_over >= self.track_thresh_ticks:
            self.fault = True
            self.fault_reason = (
                f"安全监督: J{worst_err_j} 跟踪误差 {worst_err:.1f}° > "
                f"{self.track_err_limit_deg:.1f}° (≥{self.track_window_ms:.0f}ms)"
            )
            return True
        if self._tau_over >= self.tau_thresh_ticks:
            self.fault = True
            self.fault_reason = (
                f"安全监督: J{worst_tau_j} 力矩 {worst_tau:.1f}Nm > "
                f"{worst_tau_lim:.1f}Nm (≥{self.tau_window_ms:.0f}ms)"
            )
            return True
        return False

    def status(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "fault": self.fault,
            "reason": self.fault_reason,
            "thermal_scale": float(self.thermal_scale),
            "track_err_max_deg": float(self.last_track_err_deg),
            "tau_max_ratio": float(self.last_tau_ratio),
            "tmos_max_c": None if not math.isfinite(self.last_tmos_max) else float(self.last_tmos_max),
        }


# ─────────────────────────────────────────────────────────────
# B601-RS 独立 200Hz MIT 实时控制器（直连 motorbridge）
# ─────────────────────────────────────────────────────────────
class ReBotRSMITController:
    """Seeed reBot Arm B601-RS 独立 200Hz MIT 控制器（P0~P4 完整内置）。"""

    EXT_PHASE = "P4"
    _IO_LOCK_SHUTDOWN_TIMEOUT_S = 2.0
    _TEMP_ALARM_HYSTERESIS_C = 2.0
    _FEEDBACK_ERROR_DISABLE_S = 1.0
    _SHUTDOWN_SEND_RETRY_S = 0.02

    def __init__(
        self,
        config_path: Optional[str] = None,
        *,
        hw_cfg: Optional[RSHardwareConfig] = None,
        ext_cfg: Optional[dict[str, Any]] = None,
        controller_factory: Optional[Callable[[str], Any]] = None,
    ) -> None:
        loaded_hw, loaded_ext = load_rs_mit_config(config_path)
        self.config: RSHardwareConfig = hw_cfg if hw_cfg is not None else loaded_hw
        self.ext_cfg: dict[str, Any] = (
            validate_rs_mit_ext_cfg(ext_cfg) if ext_cfg is not None else loaded_ext
        )
        self._controller_factory = controller_factory

        self.channel = self.config.channel
        self.control_hz = float(self.config.control_hz)
        self.telemetry_hz = float(self.config.telemetry_hz)

        self.temp_alarm_c = float(self.config.temperatures.alarm_c)
        self.temp_return_zero_c = float(self.config.temperatures.return_zero_c)
        self.temp_disconnect_c = float(self.config.temperatures.disconnect_c)

        self.motor_count = len(self.config.motors)
        self.controller: Any = None
        self.motors: list[Any] = []

        self.target_positions = [0.0 for _ in range(self.motor_count)]
        self.command_positions = [0.0 for _ in range(self.motor_count)]
        self.max_speeds_rad_s = [math.radians(20.0) for _ in range(self.motor_count)]

        self.target_lock = threading.RLock()
        self.io_lock = threading.RLock()
        self.shutdown_lock = threading.Lock()

        self.worker_stop_event = threading.Event()
        self.abort_return_event = threading.Event()
        self.shutdown_done_event = threading.Event()

        self.control_thread: Optional[threading.Thread] = None
        self.telemetry_thread: Optional[threading.Thread] = None
        self.shutdown_thread: Optional[threading.Thread] = None
        self.keyboard_listener: Any = None

        self.started = False
        self.shutdown_started = False
        self.return_to_zero_requested = True
        self.thermal_return_requested = False
        self.emergency_disconnect_requested = False
        self.shutdown_reason = ""
        self.signal_count = 0
        self.last_error: Optional[Exception] = None

        self.last_temperatures: list[Optional[float]] = [None for _ in range(self.motor_count)]
        self._temp_alarm_active = [False for _ in range(self.motor_count)]

        # P0/P4 状态机
        rate = self.control_hz
        self._fault_monitor = CanFaultMonitor(self.ext_cfg["can_fault_window_s"], rate)
        self._supervisor = SafetySupervisor(self.ext_cfg, rate)
        self._supervisor_lock = threading.Lock()

        # P1 速度前馈状态
        self._target_velocities = [0.0 for _ in range(self.motor_count)]
        self._command_velocities = [0.0 for _ in range(self.motor_count)]
        self._mit_vel_mode = False
        self._last_mit_update_t = -math.inf
        self._watchdog_trips = 0
        self._watchdog_was_active = False
        self._last_dq_send_max = 0.0

        # P3 重力前馈与阻尼调度状态
        self._gravity_fn: Optional[Callable[[Sequence[float]], Sequence[float]]] = None
        self._gravity_start_t: Optional[float] = None
        self._gravity_errors = 0
        self._gravity_error_logged = False
        self._tau_ff_6 = [0.0] * 6
        self._reach_r = 0.15
        self._base_kp_6 = [float(self.config.motors[i].kp) for i in range(6)]
        self._base_kd_6 = [float(self.config.motors[i].kd) for i in range(6)]
        self._cur_kp_6 = list(self._base_kp_6)
        self._cur_kd_6 = list(self._base_kd_6)

        # P0 200Hz 反馈缓存状态
        self._fb_lock = threading.Lock()
        self._fb_snapshot: Optional[dict[str, Any]] = None
        self._fb_seq = 0
        self._fb_enabled = bool(self.ext_cfg["enable"] and self.ext_cfg["feedback_cache"])
        self._fb_errors = 0
        self._fb_consecutive_errors = 0
        self._fb_disable_threshold = max(2, int(round(self._FEEDBACK_ERROR_DISABLE_S * rate)))
        self._fb_last_values: Any = None
        self._fb_change_ticks = 0
        self._fb_ticks = 0
        self._shutdown_retries = max(1, int(round(self._SHUTDOWN_SEND_RETRY_S * rate)))

        # 循环性能统计与高频录波
        window = max(16, int(round(2.0 * rate)))
        self._stats_lock = threading.Lock()
        self._period_hist: deque[float] = deque(maxlen=window)
        self._send_hist: deque[float] = deque(maxlen=window)
        self._sample_hist: deque[float] = deque(maxlen=window)
        self._ticks = 0
        self._last_can_log = -math.inf
        self._suppressed_can_logs = 0
        self._can_fault_triggered = False

        self._rec_lock = threading.Lock()
        self._recording: Optional[list[tuple[Any, ...]]] = None
        self._rec_max_rows = int(round(self.ext_cfg["record_max_s"] * rate))

    # ── IO 锁辅助 ────────────────────────────────────────────
    def _try_acquire_io_lock(self, timeout: Optional[float] = None) -> bool:
        if timeout is None:
            self.io_lock.acquire()
            return True
        return self.io_lock.acquire(timeout=timeout)

    @contextmanager
    def _io_lock_guard(self, timeout: Optional[float] = None) -> Iterator[None]:
        acquired = self._try_acquire_io_lock(timeout)
        if not acquired:
            raise TimeoutError(f"等待 io_lock 超时（{timeout} 秒）")
        try:
            yield
        finally:
            self.io_lock.release()

    # ── 连接与启动 ───────────────────────────────────────────
    def connect(self) -> None:
        if self.controller is not None:
            return
        if self._controller_factory is not None:
            ctrl = self._controller_factory(self.channel)
        else:
            if Controller is None or Mode is None:
                raise ImportError("未安装 motorbridge，请运行 ./tools/setup_env.sh 安装")
            ctrl = Controller(self.channel)
        print(f"[RSController] 打开 CAN 接口: {self.channel} (host_id=0x{self.config.host_id:02X})")
        self.controller = ctrl

        try:
            for mc in self.config.motors:
                motor = self.controller.add_robstride_motor(mc.motor_id, self.config.host_id, mc.model)
                self.motors.append(motor)
                print(f"[RSController] 电机 {mc.motor_id} ({mc.name}, {mc.model}) 已注册")

            mode_mit = Mode.MIT if Mode is not None else 0
            for index, motor in enumerate(self.motors):
                with self._io_lock_guard():
                    motor.ensure_mode(mode_mit, 1000)
                time.sleep(0.05)

            with self._io_lock_guard():
                self.controller.enable_all()
            time.sleep(0.30)
            print("[RSController] 全部 7 路 RobStride 电机已使能")

            current_positions = self._read_positions_rad()
            with self.target_lock:
                self.target_positions[:] = current_positions
                self.command_positions[:] = current_positions
            print(
                f"[RSController] 上电初始角度 (deg): "
                f"{[round(math.degrees(x), 2) for x in current_positions]}"
            )
        except Exception:
            self._disable_and_close()
            raise

    def start(
        self,
        *,
        enable_esc: bool = False,
        install_signal_handlers: bool = True,
    ) -> None:
        if self.started:
            return
        self.connect()

        self.worker_stop_event.clear()
        self.abort_return_event.clear()
        self.shutdown_done_event.clear()
        self.shutdown_started = False
        self.signal_count = 0
        self.last_error = None
        self._temp_alarm_active = [False for _ in range(self.motor_count)]
        self.started = True

        self.control_thread = threading.Thread(
            target=self._control_loop,
            name="rebot-rs-mit-control",
            daemon=True,
        )
        self.control_thread.start()

        self.telemetry_thread = threading.Thread(
            target=self._temperature_loop,
            name="rebot-rs-temperature",
            daemon=True,
        )
        self.telemetry_thread.start()

        if enable_esc:
            self._start_keyboard_listener()

        if install_signal_handlers and threading.current_thread() is threading.main_thread():
            signal.signal(signal.SIGINT, self._signal_handler)
            signal.signal(signal.SIGTERM, self._signal_handler)

        ext_on = bool(self.ext_cfg["enable"])
        print(
            f"[RSController] 已启动 ({self.control_hz:.0f}Hz MIT, "
            f"过温={self.temp_alarm_c:.0f}/{self.temp_return_zero_c:.0f}/{self.temp_disconnect_c:.0f}°C, "
            f"P0~P4={'开' if ext_on else '关'}, "
            f"vel_ff={self.ext_cfg['vel_ff_enable'] and ext_on}, "
            f"grav_ff={self.ext_cfg['gravity_ff_enable'] and ext_on}, "
            f"kd_sched={self.ext_cfg['gain_sched_enable'] and ext_on}, "
            f"M6={self.ext_cfg['supervisor_enable'] and ext_on})"
        )

    # ── 外部目标接口 ─────────────────────────────────────────
    def set_joint_angles(self, angles_deg: Sequence[float]) -> None:
        if self.shutdown_started:
            return
        if len(angles_deg) != self.motor_count:
            raise ValueError(f"必须提供 {self.motor_count} 个角度，当前收到 {len(angles_deg)} 个")
        positions: list[float] = []
        for idx, angle in enumerate(angles_deg):
            val = float(angle)
            if not math.isfinite(val):
                raise ValueError(f"关节 {idx + 1} 角度无效")
            positions.append(math.radians(val))
        with self.target_lock:
            self.target_positions[:] = positions
            self._mit_vel_mode = False
            for i in range(self.motor_count):
                self._target_velocities[i] = 0.0

    def set_joint_angle(self, joint_id: int, angle_deg: float) -> None:
        if self.shutdown_started:
            return
        if not 1 <= joint_id <= self.motor_count:
            raise ValueError(f"joint_id 必须为 1-{self.motor_count}")
        val = float(angle_deg)
        if not math.isfinite(val):
            raise ValueError("angle_deg 无效")
        with self.target_lock:
            self.target_positions[joint_id - 1] = math.radians(val)
            self._target_velocities[joint_id - 1] = 0.0

    def set_joint_mit_targets(
        self,
        angles_deg: Sequence[float],
        velocities_deg_s: Optional[Sequence[float]] = None,
    ) -> None:
        """同步下发 7 路目标角度（deg）与速度前馈（deg/s），J7 恒强制纯位置。"""
        if self.shutdown_started:
            return
        with self._supervisor_lock:
            if self._supervisor.fault:
                return
        if len(angles_deg) != self.motor_count:
            raise ValueError(f"必须提供 {self.motor_count} 个角度，当前收到 {len(angles_deg)} 个")
        positions: list[float] = []
        for idx, angle in enumerate(angles_deg):
            val = float(angle)
            if not math.isfinite(val):
                raise ValueError(f"关节 {idx + 1} 角度无效")
            positions.append(math.radians(val))

        if velocities_deg_s is None:
            velocities = [0.0] * self.motor_count
            vel_mode = False
        else:
            if len(velocities_deg_s) != self.motor_count:
                raise ValueError(
                    f"必须提供 {self.motor_count} 个速度，当前收到 {len(velocities_deg_s)} 个"
                )
            velocities = []
            for idx, vel in enumerate(velocities_deg_s):
                val = float(vel)
                if not math.isfinite(val):
                    raise ValueError(f"关节 {idx + 1} 速度无效")
                velocities.append(math.radians(val))
            if self.motor_count >= 7:
                velocities[6] = 0.0
            vel_mode = True

        now = time.monotonic()
        with self.target_lock:
            self.target_positions[:] = positions
            self._target_velocities[:] = velocities
            self._mit_vel_mode = vel_mode
            self._last_mit_update_t = now

    def set_max_speeds(self, speeds_deg_s: Sequence[float]) -> None:
        if len(speeds_deg_s) != self.motor_count:
            raise ValueError(f"必须提供 {self.motor_count} 个速度")
        speeds_rad_s: list[float] = []
        for idx, speed in enumerate(speeds_deg_s):
            val = float(speed)
            if not math.isfinite(val) or val <= 0.0:
                raise ValueError(f"关节 {idx + 1} 速度必须大于 0")
            speeds_rad_s.append(math.radians(val))
        with self.target_lock:
            self.max_speeds_rad_s[:] = speeds_rad_s

    def get_target_angles(self) -> list[float]:
        with self.target_lock:
            vals = self.target_positions.copy()
        return [math.degrees(v) for v in vals]

    def get_command_angles(self) -> list[float]:
        with self.target_lock:
            vals = self.command_positions.copy()
        return [math.degrees(v) for v in vals]

    def read_joint_angles(self) -> list[float]:
        return [math.degrees(v) for v in self._read_positions_rad()]

    def disable_motors(self) -> None:
        if self.controller is None:
            return
        with self._io_lock_guard():
            self.controller.disable_all()
        print("[RSController] 所有电机已失能")

    # ── P1/P3/P4 运行时调节与遥测接口 ────────────────────────
    def set_vel_ff_enable(self, enabled: bool) -> None:
        with self.target_lock:
            self.ext_cfg["vel_ff_enable"] = bool(enabled)
            if not enabled:
                for i in range(self.motor_count):
                    self._command_velocities[i] = 0.0

    def set_gravity_torque_fn(
        self, fn: Optional[Callable[[Sequence[float]], Sequence[float]]]
    ) -> None:
        with self.target_lock:
            self._gravity_fn = fn
            self._gravity_start_t = time.monotonic() if fn is not None else None
            self._gravity_error_logged = False

    def set_gravity_ff_runtime(
        self,
        enabled: Optional[bool] = None,
        scale: Optional[Sequence[float]] = None,
    ) -> None:
        with self.target_lock:
            if enabled is not None:
                self.ext_cfg["gravity_ff_enable"] = bool(enabled)
                if enabled and self._gravity_start_t is None:
                    self._gravity_start_t = time.monotonic()
            if scale is not None:
                if len(scale) != 6:
                    raise ValueError("gravity_ff_scale 必须为 6 维")
                clean: list[float] = []
                for i, v in enumerate(scale):
                    fv = float(v)
                    hi = 0.0 if i == 0 else 0.95
                    if not math.isfinite(fv) or not 0.0 <= fv <= hi:
                        raise ValueError(f"gravity_ff_scale[{i}]={fv} 越界 [0, {hi}]")
                    clean.append(fv)
                self.ext_cfg["gravity_ff_scale"] = clean

    def set_gain_sched_enable(self, enabled: bool) -> None:
        with self.target_lock:
            self.ext_cfg["gain_sched_enable"] = bool(enabled)

    def set_reach_radius(self, reach_r_m: float) -> None:
        val = float(reach_r_m)
        if math.isfinite(val) and 0.0 <= val <= 0.60:
            with self.target_lock:
                self._reach_r = val

    def get_feedback_snapshot(self) -> Optional[dict[str, Any]]:
        with self._fb_lock:
            snap = self._fb_snapshot
        if snap is None:
            return None
        out = dict(snap)
        for key in ("q", "dq", "tau", "t_mos", "t_rotor", "status", "valid", "cmd"):
            out[key] = list(snap[key])
        out["age_s"] = time.monotonic() - snap["time"]
        return out

    def get_safety_status(self) -> dict[str, Any]:
        with self._supervisor_lock:
            return self._supervisor.status()

    def clear_safety_fault(self) -> None:
        with self._supervisor_lock:
            self._supervisor.clear_fault()

    def set_feedback_poll_once(self, enabled: bool) -> None:
        self.ext_cfg["feedback_poll_once"] = bool(enabled)

    def reset_feedback_change_stats(self) -> None:
        with self._stats_lock:
            self._period_hist.clear()
            self._send_hist.clear()
            self._sample_hist.clear()
        self._fb_last_values = None
        self._fb_change_ticks = 0
        self._fb_ticks = 0

    def start_recording(self) -> None:
        with self._rec_lock:
            self._recording = []

    def stop_recording(self) -> list[tuple[Any, ...]]:
        with self._rec_lock:
            rows = self._recording or []
            self._recording = None
        return rows

    def get_loop_stats(self) -> dict[str, Any]:
        with self._stats_lock:
            periods = list(self._period_hist)
            sends = list(self._send_hist)
            samples = list(self._sample_hist)
            ticks = self._ticks
        ms = 1000.0

        def _p(values: Sequence[float], q: float) -> Optional[float]:
            val = _percentile(values, q)
            return None if val is None else val * ms

        return {
            "phase": self.EXT_PHASE,
            "ticks": ticks,
            "period_p50_ms": _p(periods, 0.50),
            "period_p99_ms": _p(periods, 0.99),
            "period_max_ms": (max(periods) * ms) if periods else None,
            "send_p99_ms": _p(sends, 0.99),
            "sample_p99_ms": _p(samples, 0.99),
            "can_fail_total": self._fault_monitor.total_failures,
            "can_fail_max_consecutive": self._fault_monitor.max_consecutive,
            "can_fault_threshold_ticks": self._fault_monitor.threshold,
            "can_fault_triggered": self._can_fault_triggered,
            "feedback_enabled": self._fb_enabled,
            "feedback_errors": self._fb_errors,
            "feedback_change_ratio": (
                self._fb_change_ticks / max(1, self._fb_ticks - 1) if self._fb_ticks > 1 else None
            ),
            "vel_ff_enable": bool(self.ext_cfg["enable"] and self.ext_cfg["vel_ff_enable"]),
            "dq_send_max_rad_s": float(self._last_dq_send_max),
            "watchdog_trips": int(self._watchdog_trips),
            "gravity_ff_enable": bool(self.ext_cfg["enable"] and self.ext_cfg["gravity_ff_enable"]),
            "gravity_errors": int(self._gravity_errors),
            "tau_ff_nm": list(self._tau_ff_6),
            "gain_sched_enable": bool(self.ext_cfg["enable"] and self.ext_cfg["gain_sched_enable"]),
            "cur_kp": list(self._cur_kp_6),
            "cur_kd": list(self._cur_kd_6),
            "safety": self.get_safety_status(),
        }

    # ── 200Hz MIT 实时控制主循环 ─────────────────────────────
    def _compute_p3_step_locked(
        self,
        commands: Sequence[float],
        now_mono: float,
        period: float,
        shutdown_mode: bool = False,
    ) -> tuple[list[float], list[float], list[float]]:
        q_eval = list(commands[:6])
        if not shutdown_mode:
            with self._fb_lock:
                snap = self._fb_snapshot
            if (
                snap is not None
                and all(snap["valid"][:6])
                and (now_mono - snap["time"]) <= self.ext_cfg["feedback_max_age_s"]
            ):
                q_eval = list(snap["q"][:6])
        if self._gravity_start_t is None:
            ramp = 0.0
        else:
            ramp = min(
                1.0,
                max(0.0, now_mono - self._gravity_start_t)
                / max(float(self.ext_cfg["gravity_ff_ramp_in_s"]), 1e-3),
            )
        ext_on = bool(self.ext_cfg["enable"])
        tau_6, g_err = compute_p3_gravity_tau(
            q_eval,
            self._tau_ff_6,
            self._gravity_fn,
            period,
            enabled=bool(ext_on and self.ext_cfg["gravity_ff_enable"]),
            scales=self.ext_cfg["gravity_ff_scale"],
            max_nm=self.ext_cfg["gravity_ff_max_nm"],
            ramp_factor=ramp,
            slew_nm_s=self.ext_cfg["gravity_ff_slew_nm_s"],
        )
        self._tau_ff_6 = tau_6
        if g_err:
            self._gravity_errors += 1
            if not self._gravity_error_logged:
                self._gravity_error_logged = True
                print("\n[RSController] ⚠️ 重力补偿回调异常，已平滑归零 tau_ff（不影响位置控制）")

        kp_6, kd_6 = compute_p3_gains(
            self._base_kp_6,
            self._base_kd_6,
            self._cur_kp_6,
            self._cur_kd_6,
            self._reach_r,
            period,
            enabled=bool(ext_on and self.ext_cfg["gain_sched_enable"] and not shutdown_mode),
            r_min=self.ext_cfg["gain_sched_r_min_m"],
            r_max=self.ext_cfg["gain_sched_r_max_m"],
            kd_near=self.ext_cfg["gain_sched_kd_near"],
            kd_far=self.ext_cfg["gain_sched_kd_far"],
            kp_scale=self.ext_cfg["gain_sched_kp_scale"],
            slew_per_s=self.ext_cfg["gain_sched_slew_per_s"],
        )
        self._cur_kp_6 = kp_6
        self._cur_kd_6 = kd_6
        return list(tau_6), list(kp_6), list(kd_6)

    def _control_loop(self) -> None:
        period = 1.0 / self.control_hz
        next_tick = time.perf_counter()
        last_tick: Optional[float] = None

        while not self.worker_stop_event.is_set():
            tick_start = time.perf_counter()
            now_mono = time.monotonic()
            with self.target_lock:
                ext_on = bool(self.ext_cfg["enable"])
                vel_ff_on = bool(ext_on and self.ext_cfg["vel_ff_enable"])
                foh_on = bool(self.ext_cfg["foh_interp_enable"])
                foh_dt = float(self.ext_cfg["foh_upper_dt_s"])
                wd_s = float(self.ext_cfg["vel_ff_watchdog_s"])
                wd_tau = float(self.ext_cfg["vel_ff_watchdog_tau_s"])
                lpf_tau = float(self.ext_cfg["vel_ff_lpf_tau_s"])
                scales = self.ext_cfg["vel_ff_scale"]
                elapsed = (
                    now_mono - self._last_mit_update_t
                    if math.isfinite(self._last_mit_update_t)
                    else math.inf
                )
                any_wd = False
                for index in range(self.motor_count):
                    is_arm_joint = index < 6
                    new_cmd, new_dq, wd_act = compute_p1_mit_step(
                        self.target_positions[index],
                        self.command_positions[index],
                        self._target_velocities[index] if is_arm_joint else 0.0,
                        self._command_velocities[index] if is_arm_joint else 0.0,
                        self.max_speeds_rad_s[index],
                        period,
                        elapsed,
                        vel_ff_enable=(vel_ff_on and is_arm_joint),
                        vel_ff_scale=(scales[index] if is_arm_joint else 0.0),
                        foh_enable=(foh_on and is_arm_joint),
                        foh_upper_dt_s=foh_dt,
                        watchdog_s=wd_s,
                        watchdog_tau_s=wd_tau,
                        lpf_tau_s=lpf_tau,
                        mit_vel_mode=(self._mit_vel_mode and is_arm_joint),
                    )
                    self.command_positions[index] = new_cmd
                    self._command_velocities[index] = new_dq if is_arm_joint else 0.0
                    any_wd = any_wd or wd_act
                if any_wd and not self._watchdog_was_active:
                    self._watchdog_trips += 1
                self._watchdog_was_active = any_wd
                commands = self.command_positions.copy()
                velocities = self._command_velocities.copy()
                self._last_dq_send_max = (
                    max(abs(v) for v in velocities[:6]) if len(velocities) >= 6 else 0.0
                )
                tau_6, kp_6, kd_6 = self._compute_p3_step_locked(
                    commands, now_mono, period, shutdown_mode=False
                )

            try:
                send_s, sample_s = self._send_and_sample(
                    commands, velocities, tau_6=tau_6, kp_6=kp_6, kd_6=kd_6
                )
            except Exception as error:  # noqa: BLE001
                self.last_error = error
                fault = self._fault_monitor.record_failure()
                self._log_can_error(error, fault)
                if fault:
                    self._trigger_can_fault(error)
                    return
            else:
                self._fault_monitor.record_success()
                with self._stats_lock:
                    self._send_hist.append(send_s)
                    if sample_s is not None:
                        self._sample_hist.append(sample_s)

            with self._stats_lock:
                if last_tick is not None:
                    self._period_hist.append(tick_start - last_tick)
                self._ticks += 1
            last_tick = tick_start

            next_tick += period
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()

    def _send_mit_locked(
        self,
        commands: Sequence[float],
        velocities: Optional[Sequence[float]],
        tau_6: Optional[Sequence[float]],
        kp_6: Optional[Sequence[float]],
        kd_6: Optional[Sequence[float]],
    ) -> None:
        for index, motor in enumerate(self.motors):
            mc = self.config.motors[index]
            if index < 6:
                dq_i = float(velocities[index]) if velocities is not None else 0.0
                kp_i = float(kp_6[index]) if kp_6 is not None else float(mc.kp)
                kd_i = float(kd_6[index]) if kd_6 is not None else float(mc.kd)
                tau_i = float(tau_6[index]) if tau_6 is not None else 0.0
            else:
                # J7 夹爪恒为纯位置基线 MIT
                dq_i, kp_i, kd_i, tau_i = 0.0, float(mc.kp), float(mc.kd), 0.0
            motor.send_mit(float(commands[index]), dq_i, kp_i, kd_i, tau_i)

    def _send_and_sample(
        self,
        commands: Sequence[float],
        velocities: Optional[Sequence[float]] = None,
        tau_6: Optional[Sequence[float]] = None,
        kp_6: Optional[Sequence[float]] = None,
        kd_6: Optional[Sequence[float]] = None,
    ) -> tuple[float, Optional[float]]:
        with self._io_lock_guard():
            t0 = time.perf_counter()
            self._send_mit_locked(commands, velocities, tau_6, kp_6, kd_6)
            t1 = time.perf_counter()
            sample_s = None
            if self._fb_enabled:
                self._sample_feedback_locked(commands)
                sample_s = time.perf_counter() - t1
        return t1 - t0, sample_s

    def _send_shutdown_mit_positions(
        self,
        positions_rad: Sequence[float],
        *,
        lock_timeout: Optional[float] = None,
    ) -> None:
        """安全停机回零发送：强制 dq_des=0、重力沿 smoothstep 轨迹平滑淡出、瞬时异常重试。"""
        period = 1.0 / self.control_hz
        now_mono = time.monotonic()
        with self.target_lock:
            tau_6, kp_6, kd_6 = self._compute_p3_step_locked(
                positions_rad, now_mono, period, shutdown_mode=True
            )
        last_error: Optional[Exception] = None
        for _attempt in range(self._shutdown_retries):
            try:
                with self._io_lock_guard(lock_timeout):
                    self._send_mit_locked(positions_rad, None, tau_6, kp_6, kd_6)
                return
            except TimeoutError:
                raise
            except Exception as error:  # noqa: BLE001
                last_error = error
                time.sleep(0.001)
        if last_error is not None:
            raise last_error

    def _sample_feedback_locked(self, commands: Sequence[float]) -> None:
        if self.ext_cfg.get("feedback_poll_once", False) and self.controller is not None:
            try:
                self.controller.poll_feedback_once()
            except Exception:  # noqa: BLE001
                pass
        n = len(self.motors)
        q = [math.nan] * n
        dq = [math.nan] * n
        tau = [math.nan] * n
        t_mos = [math.nan] * n
        t_rotor = [math.nan] * n
        status = [-1] * n
        valid = [False] * n
        error_seen = False

        for index, motor in enumerate(self.motors):
            try:
                state = motor.get_state()
            except Exception:  # noqa: BLE001
                error_seen = True
                continue
            if state is None:
                continue
            try:
                values = (
                    float(state.pos),
                    float(state.vel),
                    float(state.torq),
                    float(state.t_mos),
                    float(getattr(state, "t_rotor", math.nan)),
                )
            except (TypeError, ValueError, AttributeError):
                error_seen = True
                continue
            if not all(math.isfinite(v) for v in values[:3]):
                continue
            q[index], dq[index], tau[index], t_mos[index], t_rotor[index] = values
            status[index] = int(getattr(state, "status_code", -1))
            valid[index] = True

        if error_seen:
            self._fb_errors += 1
            self._fb_consecutive_errors += 1
            if self._fb_consecutive_errors >= self._fb_disable_threshold:
                self._fb_enabled = False
                print("\n[RSController] 反馈缓存连续读取异常超过 1s，已自动停用（不影响 MIT 与温保）")
        else:
            self._fb_consecutive_errors = 0

        current = (tuple(q), tuple(dq), tuple(tau))
        self._fb_ticks += 1
        if self._fb_last_values is not None and current != self._fb_last_values:
            self._fb_change_ticks += 1
        self._fb_last_values = current

        now = time.monotonic()
        self._fb_seq += 1
        snapshot = {
            "time": now,
            "seq": self._fb_seq,
            "q": q,
            "dq": dq,
            "tau": tau,
            "t_mos": t_mos,
            "t_rotor": t_rotor,
            "status": status,
            "valid": valid,
            "cmd": list(commands),
        }
        with self._fb_lock:
            self._fb_snapshot = snapshot

        if not self.shutdown_started and self.ext_cfg["enable"]:
            with self._supervisor_lock:
                newly_faulted = self._supervisor.update(
                    commands[:6], q[:6], tau[:6], t_mos, valid[:6]
                )
                fault_reason = self._supervisor.fault_reason
            if newly_faulted:
                with self.target_lock:
                    for idx in range(min(6, self.motor_count)):
                        self.target_positions[idx] = self.command_positions[idx]
                        self._target_velocities[idx] = 0.0
                        self._command_velocities[idx] = 0.0
                    self._mit_vel_mode = False
                print(
                    f"\n[RSController] ⚠️ {fault_reason} —— "
                    "已立即软停机保持当前位姿（重力补偿维持，按 g 可恢复）"
                )

        if self._recording is not None:
            with self._rec_lock:
                rec = self._recording
                if rec is not None and len(rec) < self._rec_max_rows:
                    rec.append((now, list(commands), q, dq, tau))

    # ── 位置读取与 2Hz 三级过温监控 ──────────────────────────
    def _read_positions_rad(self) -> list[float]:
        """读取 RobStride 机械角参数 0x7019（rad）。"""
        positions: list[float] = []
        with self._io_lock_guard():
            for index, motor in enumerate(self.motors):
                try:
                    pos = motor.robstride_get_param_f32(0x7019, timeout_ms=500)
                    positions.append(float(pos))
                except Exception as error:
                    mid = self.config.motors[index].motor_id
                    raise RuntimeError(f"电机 {mid} 位置读取失败: {error}") from error
        return positions

    @staticmethod
    def _safe_float(value: Any) -> Optional[float]:
        try:
            num = float(value)
        except (TypeError, ValueError):
            return None
        return num if math.isfinite(num) else None

    def _read_temperatures_once(self) -> list[Optional[float]]:
        temperatures: list[Optional[float]] = []
        with self._io_lock_guard():
            for motor in self.motors:
                try:
                    motor.request_feedback()
                except Exception:
                    pass
                try:
                    if self.controller is not None:
                        self.controller.poll_feedback_once()
                except Exception:
                    pass
                try:
                    state = motor.get_state()
                except Exception:
                    state = None
                if state is None:
                    temperatures.append(None)
                else:
                    temperatures.append(self._safe_float(getattr(state, "t_mos", None)))
        self.last_temperatures = temperatures
        return temperatures

    def _report_temperature_alerts(self, temperatures: Sequence[Optional[float]]) -> None:
        hyst = self._TEMP_ALARM_HYSTERESIS_C
        for idx, temp in enumerate(temperatures):
            if temp is None:
                continue
            mid = self.config.motors[idx].motor_id
            if temp >= self.temp_alarm_c:
                if not self._temp_alarm_active[idx]:
                    self._temp_alarm_active[idx] = True
                    print(f"\n[RSController] 温度报警: 电机 {mid} MOS={temp:.1f}°C，机械臂继续运行")
            elif temp < self.temp_alarm_c - hyst:
                self._temp_alarm_active[idx] = False

    def _temperature_loop(self) -> None:
        period = 1.0 / self.telemetry_hz
        while not self.worker_stop_event.wait(period):
            try:
                temperatures = self._read_temperatures_once()
            except Exception as error:
                print(f"\n[RSController] 温度反馈读取失败: {error}")
                continue
            valid_temps = [(i, t) for i, t in enumerate(temperatures) if t is not None]
            if not valid_temps:
                continue
            hottest_idx, hottest_temp = max(valid_temps, key=lambda item: item[1])
            hottest_mid = self.config.motors[hottest_idx].motor_id

            if hottest_temp >= self.temp_disconnect_c:
                print(
                    f"\n[RSController] 紧急断开: 电机 {hottest_mid} MOS={hottest_temp:.1f}°C，立即失能"
                )
                self.request_stop(
                    reason=f"电机 {hottest_mid} 达到紧急断开温度 ({hottest_temp:.1f}°C)",
                    return_to_zero=False,
                    emergency=True,
                    wait=False,
                )
                return
            if hottest_temp >= self.temp_return_zero_c:
                print(
                    f"\n[RSController] 高温回零: 电机 {hottest_mid} MOS={hottest_temp:.1f}°C，缓慢回零"
                )
                self.request_stop(
                    reason=f"电机 {hottest_mid} 达到高温回零阈值 ({hottest_temp:.1f}°C)",
                    return_to_zero=True,
                    thermal=True,
                    wait=False,
                )
                return
            self._report_temperature_alerts(temperatures)

    # ── CAN 故障告警、Esc 与 Ctrl+C ──────────────────────────
    def _log_can_error(self, error: Exception, fault: bool) -> None:
        now = time.monotonic()
        interval = float(self.ext_cfg["can_error_log_interval_s"])
        if fault or now - self._last_can_log >= interval:
            extra = (
                f"（期间另有 {self._suppressed_can_logs} 次未打印）"
                if self._suppressed_can_logs
                else ""
            )
            print(
                f"\n[RSController] CAN 发送异常 连续{self._fault_monitor.consecutive}/"
                f"{self._fault_monitor.threshold} 次: {error}{extra}"
            )
            self._last_can_log = now
            self._suppressed_can_logs = 0
        else:
            self._suppressed_can_logs += 1

    def _trigger_can_fault(self, error: Exception) -> None:
        self._can_fault_triggered = True
        action = str(self.ext_cfg["can_fault_action"])
        window_ms = self._fault_monitor.threshold * 1000.0 / self.control_hz
        print(
            f"\n[RSController] CAN 通信连续失败 ≥{window_ms:.0f}ms，判定通信故障；"
            + ("尝试缓慢回零后失能" if action == "return_to_zero" else "立即失能")
        )
        self.request_stop(
            reason=f"CAN 通信连续失败 ({self._fault_monitor.consecutive} 次): {error}",
            return_to_zero=(action == "return_to_zero"),
            emergency=(action == "emergency"),
            wait=False,
        )

    def _start_keyboard_listener(self) -> None:
        if _pynput_keyboard is None:
            return
        try:
            self.keyboard_listener = _pynput_keyboard.Listener(on_press=self._on_key_press)
            self.keyboard_listener.start()
        except Exception:
            self.keyboard_listener = None

    def _on_key_press(self, key: Any) -> Optional[bool]:
        if _pynput_keyboard is not None and key == _pynput_keyboard.Key.esc:
            print("\n[RSController] 检测到 Esc，正在缓慢回到零点")
            self.request_stop(reason="用户按下 Esc", return_to_zero=True, wait=False)
            return False
        return None

    def _signal_handler(self, signum: int, _frame: Any) -> None:
        self.signal_count += 1
        if self.signal_count == 1:
            print(f"\n[RSController] 收到信号 {signum}，正在缓慢回到零点")
            self.request_stop(reason=f"收到信号 {signum}", return_to_zero=True, wait=False)
        else:
            print("\n[RSController] 第二次收到 Ctrl+C，立即中止回零并失能")
            self.abort_return_event.set()
            self.worker_stop_event.set()

    # ── 安全停机与 smoothstep 缓慢回零 ───────────────────────
    def request_stop(
        self,
        *,
        reason: str = "调用 stop()",
        return_to_zero: bool = True,
        thermal: bool = False,
        emergency: bool = False,
        wait: bool = False,
    ) -> None:
        with self.shutdown_lock:
            if not self.shutdown_started:
                self.shutdown_started = True
                self.shutdown_reason = reason
                self.return_to_zero_requested = bool(return_to_zero)
                self.thermal_return_requested = bool(thermal)
                self.emergency_disconnect_requested = bool(emergency)
                self.shutdown_thread = threading.Thread(
                    target=self._shutdown_worker,
                    name="rebot-rs-safe-shutdown",
                    daemon=True,
                )
                self.shutdown_thread.start()
        if (
            wait
            and self.shutdown_thread is not None
            and threading.current_thread() is not self.shutdown_thread
        ):
            self.shutdown_done_event.wait()

    def stop(self, *, return_to_zero: bool = True, wait: bool = True) -> None:
        self.request_stop(
            reason="程序调用 arm.stop()",
            return_to_zero=return_to_zero,
            wait=wait,
        )

    def _shutdown_worker(self) -> None:
        print(f"\n[RSController] 安全停机原因: {self.shutdown_reason}")
        self.worker_stop_event.set()
        current_thread = threading.current_thread()
        for name, thread in (("control", self.control_thread), ("temperature", self.telemetry_thread)):
            if thread is not None and thread is not current_thread and thread.is_alive():
                thread.join(timeout=3.0)
                if thread.is_alive():
                    print(f"[RSController] {name} 线程 3s 内未退出，继续安全关机")

        if self.keyboard_listener is not None:
            try:
                self.keyboard_listener.stop()
            except Exception:
                pass

        try:
            if (
                self.return_to_zero_requested
                and not self.emergency_disconnect_requested
                and self.controller is not None
                and self.motors
            ):
                self._safe_return_to_zero()
        except Exception as error:
            print(f"\n[RSController] 回零异常: {error}")
        finally:
            self._disable_and_close()
            self.started = False
            self.shutdown_done_event.set()
            print("[RSController] 安全停机流程完成")

    def _safe_return_to_zero(self) -> None:
        try:
            start_positions = self._read_positions_rad()
        except Exception as error:
            print(f"[RSController] 无法读取实际角度，改用当前指令角回零: {error}")
            with self.target_lock:
                start_positions = self.command_positions.copy()

        target_positions = [0.0 for _ in range(self.motor_count)]
        rz = self.config.return_zero
        speed_deg_s = rz.thermal_max_speed_deg_s if self.thermal_return_requested else rz.max_speed_deg_s
        required_times = [1.5 * abs(math.degrees(p)) / speed_deg_s for p in start_positions]
        duration_s = max(rz.min_time_s, *required_times)
        steps = max(2, int(round(duration_s * self.control_hz)))

        print(
            f"[RSController] 正在缓慢回零: 预计 {duration_s:.1f}s，"
            f"峰值速度 ≤ {speed_deg_s:.1f}°/s（再次 Ctrl+C 可立即中止）"
        )
        period = 1.0 / self.control_hz
        next_tick = time.perf_counter()
        last_thermal_check = -math.inf
        completed = True

        for step_index in range(1, steps + 1):
            if self.abort_return_event.is_set():
                print("\n[RSController] 用户中止回零")
                completed = False
                break

            now = time.monotonic()
            if self.thermal_return_requested and now - last_thermal_check >= 1.0 / self.telemetry_hz:
                last_thermal_check = now
                temps = self._read_temperatures_once()
                valid_temps = [(i, t) for i, t in enumerate(temps) if t is not None]
                if valid_temps:
                    h_idx, h_temp = max(valid_temps, key=lambda item: item[1])
                    if h_temp >= self.temp_disconnect_c:
                        mid = self.config.motors[h_idx].motor_id
                        print(f"\n[RSController] 回零途中电机 {mid} MOS={h_temp:.1f}°C，立即中止回零")
                        completed = False
                        break

            alpha = _smoothstep01(step_index / steps)
            commands = [s + alpha * (t - s) for s, t in zip(start_positions, target_positions)]
            try:
                self._send_shutdown_mit_positions(
                    commands, lock_timeout=self._IO_LOCK_SHUTDOWN_TIMEOUT_S
                )
            except TimeoutError as error:
                print(f"\n[RSController] 回零 I/O 锁超时，中止回零: {error}")
                completed = False
                break

            with self.target_lock:
                self.command_positions[:] = commands
                self.target_positions[:] = commands

            next_tick += period
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

        if completed:
            try:
                self._send_shutdown_mit_positions(
                    target_positions, lock_timeout=self._IO_LOCK_SHUTDOWN_TIMEOUT_S
                )
            except TimeoutError as error:
                print(f"\n[RSController] 最终零点发送超时: {error}")
                completed = False

        if completed:
            with self.target_lock:
                self.command_positions[:] = target_positions
                self.target_positions[:] = target_positions
            time.sleep(rz.settle_time_s)
            print("[RSController] 已到达零位并完成保持，准备失能")

    def _disable_and_close(self) -> None:
        if self.controller is not None:
            acquired = self._try_acquire_io_lock(self._IO_LOCK_SHUTDOWN_TIMEOUT_S)
            if acquired:
                try:
                    self.controller.disable_all()
                    print("[RSController] 全部电机已失能")
                except Exception as error:
                    print(f"[RSController] disable_all 失败: {error}")
                finally:
                    self.io_lock.release()
            else:
                print("[RSController] io_lock 不可用，跳过 disable_all")

        for motor in self.motors:
            try:
                motor.close()
            except Exception:
                pass
        self.motors.clear()

        if self.controller is not None:
            for fn_name in ("shutdown", "close"):
                fn = getattr(self.controller, fn_name, None)
                if callable(fn):
                    try:
                        fn()
                    except Exception:
                        pass
        self.controller = None

    @property
    def is_stopped(self) -> bool:
        return self.shutdown_done_event.is_set()

    def wait_until_stopped(self, timeout: Optional[float] = None) -> bool:
        return self.shutdown_done_event.wait(timeout)
