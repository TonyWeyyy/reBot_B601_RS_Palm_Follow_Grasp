#!/usr/bin/env python3
"""
Seeed reBot Arm B601-RS + Orbbec Gemini 305 项目环境与静态资产自检工具
======================================================================
可在任意拷贝后的项目目录内运行，逐项核查：
  1. Python 版本与 ROS 环境变量冲突；
  2. 全部主控 Python 依赖（numpy / yaml / cv2+aruco / scipy / pynput /
     mediapipe / pinocchio / motorbridge / pyorbbecsdk）；
  3. 项目内置全部静态资产（官方刚性 RS URDF、柔性夹爪 RS URDF、相机壳3.0.3MF / STL、
     柔性夹爪 STEP、MediaPipe 模型、配置文件与标定产物、内置 SDK wheel 与 Orbbec udev 规则）；
  4. MuJoCo Studio 4.0+ 数字孪生 Python 环境（优先检查项目内 .venv_mujoco）；
  5. Linux 硬件权限与外设就绪度（SocketCAN can0 / Orbbec USB，仅提示不判失败）。
"""

from __future__ import annotations

import grp
import importlib
import importlib.metadata
import importlib.util
import os
import pwd
import shutil
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _ok(msg: str) -> None:
    print(f"  [OK]   {msg}")


def _warn(msg: str) -> None:
    print(f"  [WARN] {msg}")


def _fail(msg: str) -> None:
    print(f"  [FAIL] {msg}")


def _find_mujoco_python() -> Path | None:
    candidates: list[Path] = []
    env_py = os.environ.get("MUJOCO_STUDIO_PYTHON", "").strip()
    if env_py:
        candidates.append(Path(env_py))
    candidates.extend(
        [
            PROJECT_ROOT / ".venv_mujoco" / "bin" / "python",
            PROJECT_ROOT / ".venv" / "bin" / "python",
            Path(sys.executable),
            Path.home() / ".venvs" / "mujoco-studio" / "bin" / "python",
        ]
    )
    seen: set[str] = set()
    for p in candidates:
        sp = str(p)
        if sp in seen or not p.is_file() or not os.access(p, os.X_OK):
            continue
        seen.add(sp)
        try:
            r = subprocess.run(
                [sp, "-c", "import mujoco; from mujoco import studio; print(mujoco.__version__)"],
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )
            if r.returncode == 0:
                return p
        except Exception:
            continue
    return None


def main() -> int:
    failures = 0
    print("=" * 72)
    print("  reBot Arm B601-RS + Gemini 305 独立项目环境自检")
    print(f"  项目路径: {PROJECT_ROOT}")
    print(f"  Python  : {sys.executable} ({sys.version.split()[0]})")
    print("=" * 72)

    # 1. ROS 环境变量冲突检查
    print("\n[1] 环境变量与 ROS 冲突检查")
    ros_conflict = False
    for var in ("PYTHONPATH", "LD_LIBRARY_PATH"):
        val = os.environ.get(var, "")
        if "/opt/ros" in val:
            ros_conflict = True
            _warn(
                f"{var} 包含 /opt/ros 路径，可能导致 Pinocchio 段错误。"
                "建议运行前剔除 /opt/ros。"
            )
    if not ros_conflict:
        _ok("PYTHONPATH / LD_LIBRARY_PATH 无 /opt/ros 冲突")

    # 2. 主控 Python 依赖检查
    print("\n[2] 主控核心 Python 依赖检查")
    pkgs = [
        ("numpy", "numpy"),
        ("yaml", "PyYAML"),
        ("cv2", "opencv-contrib-python"),
        ("scipy", "scipy"),
        ("pynput", "pynput"),
        ("mediapipe", "mediapipe"),
        ("pinocchio", "pin (PyPI) / pinocchio (conda-forge)"),
        ("motorbridge", "motorbridge"),
        ("pyorbbecsdk", "pyorbbecsdk2"),
    ]
    for mod_name, dist_hint in pkgs:
        if mod_name == "pynput" and not os.environ.get("DISPLAY"):
            if importlib.util.find_spec("pynput") is not None:
                try:
                    ver = importlib.metadata.version("pynput")
                except Exception:
                    ver = "installed"
                _ok(f"{mod_name:14s} -> {ver} (无 DISPLAY/SSH 模式下仅检查包存在)")
            else:
                _fail(f"{mod_name:14s} 未安装 | 安装包: {dist_hint}")
                failures += 1
            continue
        try:
            mod = importlib.import_module(mod_name)
            ver = getattr(mod, "__version__", "installed")
            if mod_name == "cv2" and not hasattr(mod, "aruco"):
                _fail("cv2 已安装但缺少 cv2.aruco，请安装 opencv-contrib-python")
                failures += 1
            else:
                _ok(f"{mod_name:14s} -> {ver}")
        except Exception as exc:
            _fail(f"{mod_name:14s} 导入失败 ({exc}) | 安装包: {dist_hint}")
            failures += 1

    # 3. 项目内置静态资产与 SDK 包完整性
    print("\n[3] 项目内置资产与 SDK 完整性检查")
    required_files = [
        "urdf/Rebot_B601_RS/urdf/ReBot_Arm_RS.urdf",
        "urdf/Rebot_B601_RS/urdf/ReBot_Arm_RS_soft_gripper_cam.urdf",
        "urdf/Rebot_B601_RS/meshes/visual/gemini305_mount_v3.stl",
        "urdf/相机壳3.0.3MF",
        "urdf/Soft_Gripper_Finger.step",
        "urdf/Soft_Gripper_Mount.step",
        "config/hand_landmarker.task",
        "config/rs_mit.yaml",
        "config/v14.yaml",
        "config/grasp.yaml",
        "config/home_pose.json",
        "config/hand_eye_calibration.json",
        "config/joint_offset.json",
        "third_party/udev/99-obsensor-libusb.rules",
    ]
    for rel in required_files:
        p = PROJECT_ROOT / rel
        if p.is_file():
            size_kb = p.stat().st_size / 1024.0
            _ok(f"{rel} ({size_kb:.1f} KB)")
        else:
            _fail(f"缺失必要内置文件: {rel}")
            failures += 1

    wheels = sorted((PROJECT_ROOT / "third_party" / "wheels").glob("*.whl"))
    if wheels:
        _ok(f"内置 SDK Wheel ({len(wheels)} 个): {', '.join(w.name for w in wheels)}")
    else:
        _warn("third_party/wheels/ 中暂无离线 Wheel（联机可通过 pip 自动下载）")

    # 4. MuJoCo Studio 4.0+ 数字孪生环境检查
    print("\n[4] MuJoCo Studio 4.0+ 数字孪生环境检查 (v15)")
    mj_py = _find_mujoco_python()
    if mj_py is not None:
        r = subprocess.run(
            [str(mj_py), "-c", "import mujoco; print(mujoco.__version__)"],
            capture_output=True,
            text=True,
            check=False,
        )
        _ok(f"MuJoCo Studio 可用: {mj_py} (mujoco {r.stdout.strip()})")
    else:
        _warn(
            "未检测到可用的 MuJoCo Studio 4.0+ Python 环境（不影响 v14 视觉伺服，"
            "运行 ./tools/setup_env.sh --mujoco-only 可自动创建项目内 .venv_mujoco）"
        )

    # 5. Linux 硬件权限与外设状态（仅提示，不计入失败）
    print("\n[5] Linux 硬件权限与当前连接状态（仅提示）")
    udev_dst = Path("/etc/udev/rules.d/99-obsensor-libusb.rules")
    if udev_dst.is_file():
        _ok(f"系统已安装 Orbbec udev 规则: {udev_dst}")
    else:
        _warn("尚未安装 Orbbec udev 规则，接入真机前请运行: sudo ./tools/install_udev.sh")

    try:
        user = pwd.getpwuid(os.getuid()).pw_name
        user_groups = {g.gr_name for g in grp.getgrall() if user in g.gr_mem}
        user_groups.add(grp.getgrgid(os.getgid()).gr_name)
        for gname in ("video", "dialout"):
            if gname in user_groups:
                _ok(f"当前用户 '{user}' 已在 '{gname}' 组")
            else:
                _warn(f"当前用户 '{user}' 不在 '{gname}' 组（请运行 sudo ./tools/install_udev.sh 并重新登录）")
    except Exception:
        pass

    if shutil.which("ip"):
        r = subprocess.run(["ip", "link", "show", "can0"], capture_output=True, text=True, check=False)
        if r.returncode == 0 and "UP" in r.stdout:
            _ok("SocketCAN 接口 can0 已处于 UP 状态")
        elif r.returncode == 0:
            _warn("检测到 can0 但尚未 UP（运行 sudo ./tools/install_udev.sh 可拉起 1Mbps）")
        else:
            _warn("当前未检测到 SocketCAN 接口 can0（若尚未连接 B601-RS 可忽略）")

    if shutil.which("lsusb"):
        r = subprocess.run(["lsusb"], capture_output=True, text=True, check=False)
        orbbec_lines = [ln for ln in r.stdout.splitlines() if "2bc5:" in ln.lower()]
        if orbbec_lines:
            _ok(f"检测到 Orbbec USB 设备: {orbbec_lines[0].strip()}")
        else:
            _warn("当前未检测到 Orbbec USB 相机 (2bc5:*)（若尚未插相机可忽略）")

    print("\n" + "=" * 72)
    if failures == 0:
        print("  自检通过！主控依赖与项目内资产全部完整。")
        print("=" * 72)
        return 0
    print(f"  自检发现 {failures} 项失败，请按上述提示修复后重试。")
    print("=" * 72)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
