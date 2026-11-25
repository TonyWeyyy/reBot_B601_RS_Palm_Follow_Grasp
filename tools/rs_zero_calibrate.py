#!/usr/bin/env python3
"""
Seeed reBot Arm B601-RS 独立 RobStride 电机机械零位标定 / 故障诊断清除 / 随动读角工具
=====================================================================================
本脚本完全独立内置于本项目中，直接读取 ``config/rs_mit.yaml`` 并通过 ``motorbridge``
与 SocketCAN ``can0`` 上的 7 路 RobStride（RS06 × 3 + RS00 × 4）通信，零外部仓库依赖。

常用模式：
  1. 随动读角模式（默认，安全只读，不修改电机零点）：
       python3 tools/rs_zero_calibrate.py
       # 电机进入 MIT 零刚度 (kp=0, kd=0)，可手动轻托机械臂并实时查看 7 轴角度（度）

  2. 清除 RobStride 电机错误锁死并查看故障报告：
       python3 tools/rs_zero_calibrate.py --clear-error

  3. 写入机械零位（首次装机或电机机械零位漂移时使用）：
       python3 tools/rs_zero_calibrate.py --set-zero
       # ⚠️ 运行前务必先将机械臂 6 轴完全收拢贴合机械限位支架、夹爪闭合到位！
       # 写入零点后自动进入随动读角模式，确认各轴角度均显示接近 0.00°。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "rs_mit.yaml"


def _read_states_deg(ctrl, motors) -> tuple[list[float], list[float], list[int]]:
    pos_deg: list[float] = []
    t_mos: list[float] = []
    status_codes: list[int] = []
    for m in motors:
        try:
            m.request_feedback()
        except Exception:
            pass
    try:
        ctrl.poll_feedback_once()
    except Exception:
        pass
    for m in motors:
        st = None
        try:
            st = m.get_state()
        except Exception:
            st = None
        if st is None:
            pos_deg.append(float("nan"))
            t_mos.append(float("nan"))
            status_codes.append(-1)
        else:
            pos_deg.append(math.degrees(float(st.pos)))
            t_mos.append(float(getattr(st, "t_mos", float("nan"))))
            status_codes.append(int(getattr(st, "status_code", 0)))
    return pos_deg, t_mos, status_codes


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="reBot Arm B601-RS 独立 RobStride 电机零位标定 / 故障清除 / 随动读角工具"
    )
    p.add_argument(
        "--config",
        type=str,
        default=str(DEFAULT_CONFIG),
        help="RS 电机配置文件路径（默认 config/rs_mit.yaml）",
    )
    p.add_argument("--channel", type=str, default="", help="覆盖 SocketCAN 接口名（默认读取 YAML，can0）")
    p.add_argument(
        "--set-zero",
        action="store_true",
        help="将当前机械位置写入全部（或 --motors 指定）RobStride 电机零点（需二次确认）",
    )
    p.add_argument(
        "--clear-error",
        action="store_true",
        help="读取故障报告并向全部（或 --motors 指定）RobStride 电机发送 clear_error()",
    )
    p.add_argument(
        "--no-freedrive",
        action="store_true",
        help="执行完 --clear-error 或 --set-zero 后立即退出，不进入随动读角循环",
    )
    p.add_argument(
        "--motors",
        type=str,
        default="",
        help="可选：仅对指定编号电机执行零点/清错（1~7 逗号分隔，如 '1,2,3'；默认全部 7 轴）",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    cfg_path = Path(args.config)
    if not cfg_path.is_file():
        print(f"[错误] 未找到配置文件: {cfg_path}", file=sys.stderr)
        return 1

    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    hw = cfg.get("hardware", {})
    channel = args.channel or str(hw.get("channel", "can0"))
    host_id = int(hw.get("host_id", 0xFD))
    motor_specs = hw.get("motors", [])
    if len(motor_specs) != 7:
        print(f"[错误] {cfg_path} 中应配置 7 路电机，当前为 {len(motor_specs)} 路", file=sys.stderr)
        return 1

    if args.motors.strip():
        target_indices = sorted({int(x.strip()) - 1 for x in args.motors.split(",") if x.strip()})
        for idx in target_indices:
            if not 0 <= idx < 7:
                print(f"[错误] --motors 编号必须在 1~7 之间，收到: {idx + 1}", file=sys.stderr)
                return 1
    else:
        target_indices = list(range(7))

    try:
        from motorbridge import Controller, Mode
    except ImportError:
        print("[错误] 未安装 motorbridge，请先运行 ./tools/setup_env.sh 或 pip install motorbridge", file=sys.stderr)
        return 1

    if args.set_zero:
        names = ", ".join(motor_specs[i]["name"] for i in target_indices)
        print("=" * 72)
        print("  ⚠️  危险操作确认：写入 RobStride 电机机械零位 (set_zero_position)")
        print("=" * 72)
        print(f"  目标电机: {names}")
        print("  请务必确认：")
        print("    1. 机械臂 6 个关节已完全收拢并平稳贴合在机械零位限位支架上；")
        print("    2. 夹爪（J7）已闭合到机械零位；")
        print("    3. 周围无人员或障碍物，手随时可扶住机械臂。")
        ans = input("\n  请输入 YES 确认写入零位（其他任意输入取消）: ").strip()
        if ans != "YES":
            print("[已取消] 未修改任何电机零点。")
            return 0

    print(f"[连接] 打开 SocketCAN 接口: {channel} (host_id=0x{host_id:02X}) ...")
    ctrl = Controller(channel)
    motors = []
    try:
        for spec in motor_specs:
            m = ctrl.add_robstride_motor(
                int(spec["motor_id"]),
                host_id,
                str(spec["model"]),
            )
            motors.append(m)

        if args.clear_error or args.set_zero:
            print("[清错] 正在检查并清除目标电机错误锁死...")
            for idx in target_indices:
                spec = motor_specs[idx]
                mid = int(spec["motor_id"])
                try:
                    if hasattr(motors[idx], "robstride_get_fault_report"):
                        rep = motors[idx].robstride_get_fault_report()
                        if rep:
                            print(f"  - {spec['name']} (ID={mid}) 故障报告: {rep}")
                except Exception:
                    pass
                try:
                    motors[idx].clear_error()
                    print(f"  - {spec['name']} (ID={mid}): clear_error OK")
                except Exception as exc:
                    print(f"  - {spec['name']} (ID={mid}): 警告 ({exc})")
            time.sleep(0.2)

        if args.set_zero:
            print("\n[写零] 正在逐轴写入机械零位（请勿触碰机械臂）...")
            for idx in target_indices:
                spec = motor_specs[idx]
                mid = int(spec["motor_id"])
                print(f"  -> {spec['name']} (ID={mid}, {spec['model']}) ...", end=" ", flush=True)
                motors[idx].disable()
                time.sleep(0.1)
                motors[idx].set_zero_position()
                time.sleep(0.5)
                print("完成")
            print("[写零] 全部目标电机机械零位写入完成！\n")

        # 优先读一次 0x7019 机械角 + 反馈状态快照
        pos_deg, t_mos, codes = _read_states_deg(ctrl, motors)
        print("[当前状态快照]")
        for i, spec in enumerate(motor_specs):
            mech_str = ""
            try:
                mech_deg = math.degrees(float(motors[i].robstride_get_param_f32(0x7019, timeout_ms=300)))
                mech_str = f" | mechPos={mech_deg:+7.2f} deg"
            except Exception:
                pass
            print(
                f"  {spec['name']:8s} (ID={int(spec['motor_id'])}, {spec['model']:5s}): "
                f"pos={pos_deg[i]:+7.2f} deg{mech_str} | Tmos={t_mos[i]:4.1f} C | status={codes[i]}"
            )

        if args.no_freedrive or (args.clear_error and not args.set_zero):
            return 0

        print("\n[随动模式] 切换至 MIT 零刚度 (kp=0, kd=0)，可轻手移动各关节核对读数与方向。")
        print("  提示：B601-RS 抬起手臂时 J2、J3 角度应向正方向变化（[0°, +180°]）。")
        print("  按 Ctrl+C 安全退出并失能。\n")

        for m in motors:
            m.ensure_mode(Mode.MIT, 1000)
            time.sleep(0.03)
        ctrl.enable_all()

        header = "  ".join(f"{spec['name']:>8s}" for spec in motor_specs)
        print(header)
        print("-" * len(header))

        while True:
            for m in motors:
                m.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)
            pos_deg, _, _ = _read_states_deg(ctrl, motors)
            line = "  ".join(f"{p:+8.2f}" for p in pos_deg)
            print(f"\r{line}  (deg)", end="", flush=True)
            time.sleep(0.02)

    except (KeyboardInterrupt, EOFError):
        print("\n[退出] 收到中断信号，正在安全失能并关闭 CAN 总线...")
    finally:
        try:
            ctrl.disable_all()
        except Exception:
            pass
        for m in motors:
            try:
                m.close()
            except Exception:
                pass
        try:
            ctrl.shutdown()
        except Exception:
            pass
        try:
            ctrl.close()
        except Exception:
            pass
        print("[完成] 总线已安全关闭。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
