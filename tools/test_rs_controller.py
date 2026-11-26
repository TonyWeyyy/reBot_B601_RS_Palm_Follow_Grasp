#!/usr/bin/env python3
"""B601-RS 独立 200Hz MIT 底层控制器（rs_controller.py）与核心算法离线单元测试。"""

from __future__ import annotations

import math
import os
import sys
import time
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from rs_controller import (  # noqa: E402
    CanFaultMonitor,
    DEFAULT_RS_MIT_CONFIG_PATH,
    HARD_MAX_GRAVITY_TAU_NM,
    HARD_MOTOR_PEAK_TAU_NM,
    ReBotRSMITController,
    SafetySupervisor,
    compute_p1_mit_step,
    compute_p3_gains,
    compute_p3_gravity_tau,
    cradle_zero_fade,
    load_rs_mit_config,
    validate_rs_mit_ext_cfg,
)
from servo_timing import (  # noqa: E402
    horizontal_radial_soft_wall,
    scaled_alpha,
    scaled_decay,
)
from kalman_target import KalmanTarget3D  # noqa: E402


class _FakeState:
    def __init__(self, pos=0.0, vel=0.0, torq=0.0, t_mos=42.0, t_rotor=40.0, status_code=0):
        self.pos = pos
        self.vel = vel
        self.torq = torq
        self.t_mos = t_mos
        self.t_rotor = t_rotor
        self.status_code = status_code


class _FakeMotor:
    def __init__(self, motor_id: int, model: str) -> None:
        self.motor_id = motor_id
        self.model = model
        self.pos = 0.0
        self.vel = 0.0
        self.torq = 0.0
        self.t_mos = 42.0
        self.sent: list[tuple[float, float, float, float, float]] = []
        self.fail_next = 0
        self.closed = False

    def ensure_mode(self, _mode, _timeout_ms=1000) -> None:
        pass

    def robstride_get_param_f32(self, _param_id: int, timeout_ms: int = 500) -> float:
        return float(self.pos)

    def send_mit(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        if self.fail_next > 0:
            self.fail_next -= 1
            raise RuntimeError("simulated CAN tx drop")
        self.sent.append((pos, vel, kp, kd, tau))
        self.pos = pos
        self.vel = vel

    def request_feedback(self) -> None:
        pass

    def get_state(self) -> _FakeState:
        return _FakeState(self.pos, self.vel, self.torq, self.t_mos)

    def close(self) -> None:
        self.closed = True


class _FakeBridgeController:
    def __init__(self, channel: str) -> None:
        self.channel = channel
        self.motors: list[_FakeMotor] = []
        self.enabled = False
        self.disabled_calls = 0
        self.shutdown_calls = 0
        self.closed = False

    def add_robstride_motor(self, motor_id: int, _host_id: int, model: str) -> _FakeMotor:
        m = _FakeMotor(motor_id, model)
        self.motors.append(m)
        return m

    def enable_all(self) -> None:
        self.enabled = True

    def disable_all(self) -> None:
        self.enabled = False
        self.disabled_calls += 1

    def poll_feedback_once(self) -> None:
        pass

    def shutdown(self) -> None:
        self.shutdown_calls += 1

    def close(self) -> None:
        self.closed = True


class RSControllerConfigTests(unittest.TestCase):
    """验证 config/rs_mit.yaml 完整性与 RS06/RS00 硬件安全红线校验。"""

    def test_load_default_rs_mit_yaml(self) -> None:
        hw, ext = load_rs_mit_config(DEFAULT_RS_MIT_CONFIG_PATH)
        self.assertEqual(hw.channel, "can0")
        self.assertEqual(hw.host_id, 0xFD)
        self.assertEqual(len(hw.motors), 7)
        self.assertEqual(hw.motors[0].model, "rs-06")
        self.assertEqual(hw.motors[3].model, "rs-00")
        self.assertEqual(hw.motors[6].model, "rs-00")
        self.assertTrue(ext["enable"])
        self.assertEqual(ext["gravity_payload_mass_kg"], 0.095)
        self.assertEqual(ext["tau_limit_nm"], [18.0, 28.0, 28.0, 10.0, 10.0, 10.0])

    def test_reject_unsafe_rs_ext_configs(self) -> None:
        bad_cases = [
            {"unknown_key": 1},
            {"gravity_ff_scale": [0.2, 0.5, 0.5, 0.5, 0.0, 0.0]},  # J1 必须为 0
            {"gravity_ff_max_nm": [0.0, HARD_MAX_GRAVITY_TAU_NM[1] + 1.0, 10.0, 2.0, 1.0, 1.0]},
            {"tau_limit_nm": [18.0, 28.0, 28.0, HARD_MOTOR_PEAK_TAU_NM[3] + 1.0, 10.0, 10.0]},
            {"gain_sched_kd_far": [6.0, 14.0, 14.0, 6.0, 4.0, 4.0]},  # RS00 阻尼硬上限 5.0
            {"feedback_cache": False, "feedback_trusted": True},
            {"can_fault_window_s": 0.50},
        ]
        for bad in bad_cases:
            with self.assertRaises(ValueError):
                validate_rs_mit_ext_cfg(bad)


class RSControllerAlgorithmsTests(unittest.TestCase):
    """验证 P0/P1/P3/P4 纯算法与安全护栏。"""

    def test_can_fault_monitor_window(self) -> None:
        mon = CanFaultMonitor(0.03, 200.0)
        self.assertEqual(mon.threshold, 6)
        for _ in range(5):
            self.assertFalse(mon.record_failure())
        mon.record_success()
        self.assertEqual(mon.consecutive, 0)
        for _ in range(5):
            self.assertFalse(mon.record_failure())
        self.assertTrue(mon.record_failure())

    def test_p1_foh_and_watchdog(self) -> None:
        cmd, dq, wd = compute_p1_mit_step(
            target=0.10,
            command=0.09,
            dq_target=1.0,
            dq_prev_send=0.0,
            max_speed=2.0,
            period=0.005,
            elapsed=0.005,
            vel_ff_enable=True,
            vel_ff_scale=0.85,
            foh_enable=True,
            foh_upper_dt_s=0.010,
            watchdog_s=0.040,
            watchdog_tau_s=0.030,
            lpf_tau_s=0.0,
            mit_vel_mode=True,
        )
        self.assertFalse(wd)
        self.assertAlmostEqual(cmd, 0.095, places=6)
        self.assertAlmostEqual(dq, 0.85, places=6)

        _, dq_late, wd_late = compute_p1_mit_step(
            target=0.10,
            command=0.095,
            dq_target=1.0,
            dq_prev_send=0.85,
            max_speed=2.0,
            period=0.005,
            elapsed=0.200,
            vel_ff_enable=True,
            vel_ff_scale=0.85,
            foh_enable=True,
            foh_upper_dt_s=0.010,
            watchdog_s=0.040,
            watchdog_tau_s=0.030,
            lpf_tau_s=0.0,
            mit_vel_mode=True,
        )
        self.assertTrue(wd_late)
        self.assertLess(abs(dq_late), 0.05)

    def test_p3_gravity_cradle_fade_and_caps(self) -> None:
        self.assertEqual(cradle_zero_fade([0.0, 0.01, 0.01, 0.0, 0.0, 0.0]), 0.0)
        self.assertEqual(cradle_zero_fade([0.0, 0.35, 0.35, 0.0, 0.0, 0.0]), 1.0)
        tau, err = compute_p3_gravity_tau(
            q_eval_6=[0.0, 0.35, 0.35, 0.1, 0.0, 0.0],
            prev_tau_6=[0.0] * 6,
            gravity_fn=lambda _q: [9.0, 10.0, 10.0, 2.0, 0.0, 0.0],
            period=0.005,
            enabled=True,
            scales=[0.0, 0.65, 0.80, 0.80, 0.0, 0.0],
            max_nm=[0.0, 12.0, 12.0, 2.5, 2.0, 1.0],
            ramp_factor=1.0,
            slew_nm_s=30.0,
        )
        self.assertFalse(err)
        self.assertEqual(tau[0], 0.0)  # J1 恒为 0
        self.assertAlmostEqual(tau[1], 0.15, places=6)  # 受 slew_nm_s*period=0.15 限幅

    def test_p3_gains_rs00_clamp(self) -> None:
        kp, kd = compute_p3_gains(
            base_kp_6=[72, 216, 216, 50, 50, 50],
            base_kd_6=[3.6, 12.0, 12.0, 5.0, 4.0, 4.0],
            prev_kp_6=[72, 216, 216, 50, 50, 50],
            prev_kd_6=[3.6, 12.0, 12.0, 5.0, 4.0, 4.0],
            reach_r=0.40,
            period=1.0,
            enabled=True,
            r_min=0.18,
            r_max=0.36,
            kd_near=[4.6, 13.5, 14.0, 5.0, 4.0, 4.0],
            kd_far=[6.4, 14.5, 15.0, 5.0, 4.0, 4.0],
            kp_scale=[1.0] * 6,
            slew_per_s=20.0,
        )
        self.assertAlmostEqual(kd[0], 6.4, places=5)
        self.assertLessEqual(kd[3], 5.0)
        self.assertEqual(kp[0], 72.0)

    def test_p4_safety_supervisor_and_thermal(self) -> None:
        _, ext = load_rs_mit_config(DEFAULT_RS_MIT_CONFIG_PATH)
        sup = SafetySupervisor(ext, 200.0)
        valid = [True] * 6
        # 100°C 在 90~110°C 中点，降速倍率 = 0.70
        sup.update([0.0] * 6, [0.0] * 6, [0.0] * 6, [100.0] * 7, valid)
        self.assertAlmostEqual(sup.thermal_scale, 0.70, places=5)
        self.assertFalse(sup.fault)

        # 持续超跟踪误差 16 拍（80ms @ 200Hz）触发软停机
        tripped = False
        for _ in range(sup.track_thresh_ticks):
            tripped = sup.update([0.5, 0, 0, 0, 0, 0], [0.0] * 6, [0.0] * 6, [45.0] * 7, valid)
        self.assertTrue(tripped)
        self.assertTrue(sup.fault)
        sup.clear_fault()
        self.assertFalse(sup.fault)


class RSControllerRuntimeMockTests(unittest.TestCase):
    """基于虚拟 motorbridge 检验 200Hz 主循环、J7 隔离与安全回零停机链。"""

    def test_start_mit_j7_isolation_and_safe_shutdown(self) -> None:
        bridges: list[_FakeBridgeController] = []

        def _factory(ch: str) -> _FakeBridgeController:
            b = _FakeBridgeController(ch)
            bridges.append(b)
            return b

        hw, ext = load_rs_mit_config(DEFAULT_RS_MIT_CONFIG_PATH)
        # 缩短测试中的回零最短时长以加速离线验证
        from dataclasses import replace

        fast_hw = replace(
            hw,
            return_zero=replace(hw.return_zero, min_time_s=0.15, settle_time_s=0.02, max_speed_deg_s=180.0),
        )
        ctrl = ReBotRSMITController(hw_cfg=fast_hw, ext_cfg=ext, controller_factory=_factory)
        ctrl.start(enable_esc=False, install_signal_handlers=False)
        try:
            # 验证 J7 速度前馈被强制清零隔离
            ctrl.set_joint_mit_targets(
                [5.0, 10.0, 10.0, 0.0, 0.0, 0.0, 100.0],
                [15.0, 15.0, 15.0, 10.0, 10.0, 10.0, 99.0],
            )
            time.sleep(0.06)
            snap = ctrl.get_feedback_snapshot()
            self.assertIsNotNone(snap)
            self.assertTrue(all(snap["valid"]))
            # 检查 J7（电机 7，索引 6）发送的所有 MIT 帧 dq_des 和 tau_ff 恒为 0
            j7_sent = bridges[0].motors[6].sent
            self.assertGreater(len(j7_sent), 0)
            for _pos, vel, _kp, _kd, tau in j7_sent:
                self.assertEqual(vel, 0.0)
                self.assertEqual(tau, 0.0)
        finally:
            ctrl.stop(return_to_zero=True, wait=True)

        self.assertTrue(ctrl.is_stopped)
        self.assertGreaterEqual(bridges[0].disabled_calls, 1)
        self.assertTrue(bridges[0].closed)
        # 验证安全回零最终将所有轴送回 0.0 rad
        for m in bridges[0].motors:
            self.assertAlmostEqual(m.sent[-1][0], 0.0, places=3)


class CoreTimingAndKalmanSanityTests(unittest.TestCase):
    """保留核心时序缩放、70% 水平软墙、3D 卡尔曼 XY 子空间更新与本地 RS URDF Pinocchio 校验。"""

    def test_timing_and_soft_wall(self) -> None:
        self.assertAlmostEqual(scaled_alpha(0.50, 0.01), 0.50, places=12)
        self.assertAlmostEqual(scaled_decay(0.90, 0.02, 0.01), 0.81, places=12)
        v_out, hit = horizontal_radial_soft_wall(
            np.array([0.425, 0.0, 0.25]),
            np.array([0.50, 0.10, 0.20]),
            radius=0.42,
        )
        # 触碰 70% 水平软墙时外推径向分量归零，切向与 Z 向保留
        self.assertTrue(hit)
        self.assertAlmostEqual(float(v_out[0]), 0.0, places=6)
        self.assertAlmostEqual(float(v_out[1]), 0.10, places=6)
        self.assertAlmostEqual(float(v_out[2]), 0.20, places=6)

    def test_kalman_partial_xy_update_preserves_z(self) -> None:
        kf = KalmanTarget3D(q=1.8, r_xy=0.0023, r_z=0.0038)
        kf.update(np.array([0.10, 0.20, 0.40]), dt=0.02)
        kf.update(np.array([0.11, 0.20, 0.40]), dt=0.02)
        z_before = float(kf.position()[2])
        H_xy = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64)
        kf.update_partial(np.array([0.15, 0.22]), H_pos=H_xy, dt=0.02, r_diag=np.array([0.0023, 0.0023]))
        self.assertAlmostEqual(float(kf.position()[2]), z_before, places=4)

    def test_local_rs_urdf_loads_with_pinocchio(self) -> None:
        import pinocchio as pin
        from rebot_adapter import LOCAL_RS_URDF_PATH, B601_RS_JOINT_NAMES

        self.assertTrue(os.path.isfile(LOCAL_RS_URDF_PATH))
        self.assertTrue(os.path.abspath(LOCAL_RS_URDF_PATH).startswith(os.path.abspath(ROOT)))
        model = pin.buildModelFromUrdf(LOCAL_RS_URDF_PATH)
        data = model.createData()
        q = np.zeros(model.nq, dtype=np.float64)
        for jn in B601_RS_JOINT_NAMES:
            self.assertGreater(model.getJointId(jn), 0)
        g = pin.computeGeneralizedGravity(model, data, q)
        self.assertTrue(np.all(np.isfinite(g)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
