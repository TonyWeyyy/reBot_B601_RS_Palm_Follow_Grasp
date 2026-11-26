# AGENTS.md

## Overview
Fully self-contained eye-in-hand visual servoing + pinch grasping + bidirectional MuJoCo Studio 3D digital twin for the **Seeed reBot Arm B601-RS** (RobStride `RS06` × 3 + `RS00` × 4 on SocketCAN `can0`, `host_id=0xFD`) and **Orbbec Gemini 305** RGB-D wrist camera (mounted in 3D-printed bracket `urdf/相机壳3.0.3MF`, 0° tilt, native 60fps + hardware D2C).

This repository must remain **100% standalone and relocatable** with zero hardcoded host paths or external repository dependencies:
- Bundled hardware SDK wheels & udev rules: `third_party/wheels/` (`motorbridge`, `pyorbbecsdk2`) and `third_party/udev/99-obsensor-libusb.rules`.
- Bundled URDF/STL/3MF/STEP & vision model: `urdf/Rebot_B601_RS/` (both official rigid `ReBot_Arm_RS.urdf` and modified soft-gripper `ReBot_Arm_RS_soft_gripper_cam.urdf`, plus `gemini305_mount_v3.stl`), `urdf/相机壳3.0.3MF`, `urdf/Soft_Gripper_*.step`, and `config/hand_landmarker.task`.
- Standalone setup/check/calibration scripts: `tools/install_udev.sh`, `tools/setup_env.sh` (creates project-local `.venv/` and `.venv_mujoco/`), `tools/check_env.py`, `tools/rs_zero_calibrate.py`, `tools/run_all_tests.sh`.
- Main entrypoints (clean v14 + v15 architecture):
  - `palm_follow_v14.py`: 4-layer decoupled visual servoing + pinch grasping (`servo_v14/` — `estimator.py` → `state_machine.py` → `controller_pbvs.py` → `safety_shield.py` + `depth_guard.py` + `robot_backend.py` + non-blocking UDP `mujoco_bridge.py`, configured via immutable dataclasses in `config/v14.yaml`).
  - `mujoco_teleop_v15.py` + `servo_v14/teleop_bridge.py`: bidirectional MuJoCo ↔ B601-RS safe teleoperation over dual UDP `:19870`/`:19871` (Joint Sliders & 3D Target-Sphere DLS IK, 8° anti-jump alignment gate, 250ms watchdog, 0.42m soft wall, Z≥0.05m table guard, M6 interlock).
  - `mujoco_twin/` (`run_twin.py`, `scene_builder.py`, `sim_plugin.py`, `viewer_plugin.py`, `messages.py`) + `tools/launch_mujoco_twin.sh`: MuJoCo Studio 4.0+ 3D digital twin (`--native` / `--web`) supporting live hot-switching between `soft_gripper` (`ReBot_Arm_RS_soft_gripper_cam.urdf`) and `standard` (`ReBot_Arm_RS.urdf`) URDF variants, mounting `gemini305_mount_v3.stl` + Gemini 305 visual body + 0.35m optical aim ray on `link6`.
- Low-level RS controller:
  - `rs_controller.py` + `config/rs_mit.yaml`: standalone 200Hz RobStride `motorbridge` MIT controller with P0~P4 features built in (30ms CAN fault window, 200Hz zero-overhead `get_state()` cache, P1 velocity FF + FOH + 40ms watchdog, P3 Pinocchio gravity FF + reach-adaptive `kd`, P4 M6 collision/over-torque soft-hold + 90~110°C thermal derating + 110/125/140°C 3-level over-temp protection).
  - `rebot_adapter.py`: radian/meter/Nm adapter with isolated Pinocchio gravity model (`0.095 kg` Gemini 305 + `相机壳3.0.3MF` payload on `joint6`) and `config/joint_offset.json` FK-only correction.
- Camera:
  - `gemini305_camera.py`: Orbbec SDK v2 (`pyorbbecsdk2`) Pipeline with HW D2C alignment, native 60fps `848×480` profile selection, dynamic `depth_scale`, forced zero distortion (`dist_coeffs = np.zeros(5)`), and `latest_only=True` queue drain.

## Verification & Testing (Run Before Hardware)
```bash
# 0. Environment & bundled-asset self-check
python3 tools/check_env.py

# 1. Run all 4 offline verification suites (auto-selects .venv / .venv_mujoco)
./tools/run_all_tests.sh
```

## Critical B601-RS + Gemini 305 Hardware Facts & Safety Red Lines
1. **RS Joint Direction & Limits**: B601-RS `joint2` and `joint3` operate in positive range (`[0, +3.14]` rad; lifting up increases J2/J3). Never copy B601-DM joint angles or configs into this project.
2. **RobStride Motor Torque Ceilings**: `RS06` (J1~J3) peak = **36 Nm** (`tau_limit_nm` = `[18, 28, 28] Nm`); `RS00` (J4~J6, J7) peak = **14 Nm** (`tau_limit_nm` = `[10, 10, 10] Nm`, RS00 `kd <= 5.0`).
3. **Safe Shutdown Sequence**: Never disable motors in a non-zero / elevated pose. Every exit path executes: stop 100Hz servo → `backend.hold()` → `backend.stop()` (smoothstep return to `[0, 0, 0, 0, 0, 0]` with `cradle_zero_fade` gravity fade → settle → `disable_all()` → `shutdown()` → `close()`).
4. **Single CAN Master on `can0`**: All 7 motors (6 arm + J7 gripper) share one `ReBotRSMITController` instance on `can0`. Never open a second `motorbridge` handle.
5. **4-Layer Depth Protection Chain (Gemini 305)**: `min_valid_m = 0.07m`, `max_valid_m = 1.20m`, default target `distance = 0.35m` (`[0.10, 0.60]m`). Do not bypass `DepthProtectionChain`.
6. **Hand-Eye Recalibration Mandatory**: `config/hand_eye_calibration.json` ships with a nominal 0°-tilt placeholder (`calibrated: false`). Run `tools/handeye_joint_calib.py` and `tools/p3_gravity_verify.py` before real-robot visual servoing with Gemini 305.
7. **Workspace & Speed Limits**: Horizontal radial soft wall `<= 0.42m` (`+5cm` hard breach → return HOME), Cartesian `max_vel <= 0.85 m/s`, `HARD_MAX_JOINT_SPEED = [2.5, 2.0, 2.0, 3.0, 3.0, 2.5] rad/s` (J2/J3 `<= 2.0 rad/s`).
8. **ROS Conflict**: Strip `/opt/ros/*` from `PYTHONPATH` / `LD_LIBRARY_PATH` if Pinocchio segfaults.
