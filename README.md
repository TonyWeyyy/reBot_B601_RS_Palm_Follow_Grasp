# reBot Arm B601-RS + Orbbec Gemini 305 视觉伺服抓取与双向数字孪生系统（v14 + v15）

> **分支说明**：本分支（`feat/gemini305`）专门适配 **Seeed reBot Arm B601-RS + Orbbec Gemini 305 + [urdf/相机壳3.0.3MF](urdf/相机壳3.0.3MF)**；若使用旧版 Orbbec Gemini 2 相机与旧支架，请切换至 `main` 分支。

本工程为 **100% 独立自包含项目**：内置 B601-RS 双版本 URDF/STL 模型（**官方原版刚性夹爪** 与 **改装柔性夹爪**）、Gemini 305 外壳 3D 打印源文件（`urdf/相机壳3.0.3MF`）及其无损转换网格（`gemini305_mount_v3.stl`）、柔性夹爪 STEP 源文件、MediaPipe 离线手掌模型、RobStride `motorbridge` 与奥比中光 `pyorbbecsdk2` 硬件 SDK Wheel、Orbbec 全系 USB udev 规则、RS 独立机械零位标定工具以及一键环境搭建与自检脚本。

**一台 Gemini 305 相机、一次 MediaPipe 21 点检测、一条 `can0` 总线句柄**，即可同时实现：
- **v14 视觉伺服 + 捏合抓取（`palm_follow_v14.py`）**：发挥 Gemini 305 近距优势（理想深度 `0.07 ~ 0.50 m`），手掌 3D 移动 → 机械臂保持固定姿态与期望距离（默认 `0.35 m`，可设低至 `0.10 m`）实时跟随；拇指-食指捏合 → J7 夹爪同步开合；
- **v15 MuJoCo 双向数字孪生遥操作（`mujoco_teleop_v15.py` + `mujoco_twin/`）**：在本地原生 3D 窗口拖动关节滑块或 3D 目标球（实时 DLS IK）平滑安全驱动外部真机，支持**改装柔性夹爪 URDF** 与 **官方原版刚性夹爪 URDF** 一键热切换（均精确挂载 `gemini305_mount_v3.stl` + Gemini 305 机身 + `0.35 m` 光轴），同时 100Hz 实时镜像真机状态并支持一键回放历史高频 CSV 日志。

---

## 目录

1. [硬件清单与安全须知](#一硬件清单与安全须知)
2. [项目目录结构（全部内置依赖一览）](#二项目目录结构全部内置依赖一览)
3. [从零开始的环境配置与安装教程](#三从零开始的环境配置与安装教程)
4. [离线全量验证（无需硬件）](#四离线全量验证无需硬件)
5. [换装 Gemini 305 后的实机标定 SOP（必做）](#五换装-gemini-305-后的实机标定-sop必做)
6. [日常运行指南（v14 视觉伺服 & v15 双向遥操作）](#六日常运行指南v14-视觉伺服--v15-双向遥操作)
7. [配置文件说明与按症状调参速查](#七配置文件说明与按症状调参速查)
8. [常见故障排查 FAQ](#八常见故障排查-faq)
9. [每次运行前安全 Checklist](#九每次运行前安全-checklist)

---

## 一、硬件清单与安全须知

### 1.1 硬件组成
| 组件 | 型号 / 规格 | 说明 |
|---|---|---|
| **机械臂** | Seeed reBot Arm B601-RS | 6 轴 + J7 夹爪：J1~J3 为 RobStride `RS06`（峰值 `36 Nm`，ID `1~3`）；J4~J6 与 J7 夹爪为 `RS00`（峰值 `14 Nm`，ID `4~7`） |
| **末端夹爪** | 改装柔性夹爪 / 官方刚性夹爪 | `RS00`（ID `7`）驱动，共用 `can0` 单总线；数字孪生内置两版 URDF 支持随时切换 |
| **通信接口** | SocketCAN `can0` (1Mbps) | USB-CAN 适配器，`host_id = 0xFD`；通过 `sudo ./tools/install_udev.sh` 一键拉起 |
| **RGB-D 相机** | Orbbec Gemini 305（USB 3.0） | 超紧凑全局快门双目相机（`42×42×23 mm`，`~68 g`，基线 `18 mm`），理想工作距离 **`0.07 ~ 0.50 m`**（有效 `0.04 ~ 1.20 m`），原生支持 `848×480@60fps` 彩色/深度与硬件 D2C 对齐 |
| **相机支架** | [urdf/相机壳3.0.3MF](urdf/相机壳3.0.3MF) | 3D 打印包裹外壳（外廓 `44×58×26 mm`，内腔 `42×42×26 mm`，**0° 倾角前视**），整套腕部相机+支架载荷仅 **`0.095 kg`**（较 Gemini 2 的 `0.32 kg` 大幅减重） |
| **推荐主机** | Ubuntu 22.04 / 20.04 x86_64 | 需至少 1 个 **USB 3.0 蓝色接口**（接 Gemini 305 跑 60fps）与 1 个 USB-CAN 接口 |

### 1.2 核心安全红线
> 1. **换装新支架与相机后务必重新做手眼标定**：本分支 `config/hand_eye_calibration.json` 为 **0° 倾角名义占位外参**（`calibrated: false`），真机跟随前请务必运行 `tools/handeye_joint_calib.py` 实测外参，并用 `tools/p3_gravity_verify.py` 复核 `0.095 kg` 轻量负载下的重力前馈。
> 2. **严禁非零位直接失能**：任何退出路径都必须走完「停止伺服 → 原地保持 → smoothstep 缓慢回零（重力前馈平滑淡出至 0）→ 到位失能」。
> 3. **严禁混用 B601-DM 关节配置**：B601-RS 的 `joint2` / `joint3` 为正区间（抬臂角度增大），关节轴向、力矩上限与 DM 完全不同。
> 4. **单一 CAN 主机**：全部 7 路电机共享同一个 `ReBotRSMITController`（`can0`），严禁同时启动第二个 `motorbridge` 进程。

---

## 二、项目目录结构（全部内置依赖一览）

```text
<项目根目录>/
├── palm_follow_v14.py             ★ v14.0 视觉伺服 + 捏合抓取主程序（支持真机 / --sim / --no-arm）
├── mujoco_teleop_v15.py           ★ v15.0 MuJoCo <-> B601-RS 双向遥操作桥接主程序
├── rs_controller.py               ★ B601-RS 独立 200Hz 底层控制器（直连 motorbridge + P0~P4）
├── rebot_adapter.py               ★ RS 机械臂适配层（Pinocchio 双模型隔离 + 0.095kg 载荷 + J7 夹爪）
├── gemini305_camera.py            ★ Orbbec Gemini 305 SDK v2 采集层（848×480@60fps + HW D2C + 动态 scale）
├── palm_detection.py              MediaPipe Tasks 21点检测 + 迟滞 ROI + LK 光流桥接 + 捏合提取
├── kalman_target.py               3D 恒速卡尔曼滤波（分轴 Q + 深度冻结期 XY 子空间更新）
├── servo_timing.py                时间校准 / 静止保持 / 伸展调度 / 软墙 / 振荡阻尼等纯算法
├── servo_diagnostics.py           非阻塞高频 CSV 诊断记录器
├── requirements.txt               主控制 Python 环境依赖清单（含 PyPI `pin`、SDK、OpenCV 等）
├── requirements-mujoco.txt        MuJoCo Studio 4.0+ 数字孪生环境依赖清单
│
├── third_party/                   ★ 内置第三方硬件 SDK Wheel 与 Orbbec udev 规则（含 Gemini 305 PID）
├── servo_v14/                     ★ v14/v15 四层解耦核心包（适配 Gemini 305 0.07 ~ 1.20 m）
├── mujoco_twin/                   ★ MuJoCo Studio 4.0+ 3D 数字孪生（双 URDF + 相机壳3.0 STL + Gemini 305）
│
├── urdf/                          ★ 内置 B601-RS 双 URDF、相机壳3.0.3MF 与柔性夹爪 CAD 源文件
│   ├── 相机壳3.0.3MF                               （Gemini 305 腕部支架 3D 打印源文件）
│   ├── Soft_Gripper_Finger.step / Soft_Gripper_Mount.step
│   └── Rebot_B601_RS/
│       ├── urdf/ReBot_Arm_RS.urdf                   （官方原版刚性夹爪 URDF）
│       ├── urdf/ReBot_Arm_RS_soft_gripper_cam.urdf  （改装柔性夹爪 URDF）
│       └── meshes/                                  （含无损转换的 gemini305_mount_v3.stl 与全部视觉/碰撞 STL）
│
├── config/                        ★ 仅 3 份核心 YAML + 标定 JSON + 离线视觉模型
│   ├── rs_mit.yaml                ★ RS CAN/7路电机 + MIT P0~P4（已更新 0.095kg Gemini 305 载荷）
│   ├── v14.yaml                   ★ v14 视觉伺服、Gemini 305 深度保护（0.07~1.20m, distance=0.35m）
│   ├── grasp.yaml                 ★ J7 夹爪捏合映射与实测开合角（5.2° ~ 285.2°）
│   ├── home_pose.json             沿用 RS 已示教 HOME 位姿
│   ├── joint_offset.json          沿用 RS 6 轴关节零偏
│   ├── hand_eye_calibration.json  0° 倾角名义初始外参（换装 305 后请用 handeye_joint_calib.py 实测覆盖）
│   ├── viewpoint.json             手眼自动标定基准观察点
│   └── hand_landmarker.task       MediaPipe 手掌关键点离线推理模型（7.5 MB）
│
└── tools/                         安装、自检、标定、系统辨识与离线测试工具集
```

---

## 三、从零开始的环境配置与安装教程

```bash
cd /path/to/rebot_B601_rs_palm_follow_grasp

# 1. 安装 Orbbec udev 规则（含 Gemini 305 PID 0840/0841/0842）+ 用户组 + SocketCAN can0 (1Mbps)
sudo ./tools/install_udev.sh

# 2. 一键创建项目内双虚拟环境（.venv 主控 + .venv_mujoco 3D 孪生，优先使用内置 Wheel）
./tools/setup_env.sh

# 3. 激活主控环境并自检
source .venv/bin/activate
python3 tools/check_env.py
```

> ⚠️ **ROS 冲突提示**：若终端 `source` 过 `/opt/ros/*/setup.bash`，运行前请剔除 `/opt/ros`（`setup_env.sh` 与 `run_all_tests.sh` 已自动处理）：
> ```bash
> export PYTHONPATH=$(echo "${PYTHONPATH:-}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd:)
> export LD_LIBRARY_PATH=$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd:)
> ```

---

## 四、离线全量验证（无需硬件）

```bash
./tools/run_all_tests.sh
```
覆盖 4 套自动化测试：
1. `tools/test_rs_controller.py` — RS 底层 200Hz MIT、P0~P4、`0.095 kg` 载荷与安全回零；
2. `tools/test_v14_architecture.py` — v14 四层架构、Gemini 305 深度保护链（`0.07 ~ 1.20 m`、`distance=0.35 m`）与闭环单测；
3. `tools/test_v15_teleop.py` — v15 双向遥操作 5 道安全防线；
4. `tools/test_mujoco_twin_rs.py` — MuJoCo 4.0+ 编译双 RS URDF（`soft_gripper` & `standard`）+ `gemini305_mount_v3.stl` + Gemini 305 双目机身 + 0° 光轴校验。

离线 3D 双向仿真预览：
```bash
# 终端 1：启动 3D 窗口（默认改装柔性夹爪 URDF；加 --urdf-variant standard 切官方刚性版；窗口内可一键热切换）
./tools/launch_mujoco_twin.sh --native

# 终端 2：启动 v15 离线数字孪生桥接
source .venv/bin/activate
python3 mujoco_teleop_v15.py --sim
```

---

## 五、换装 Gemini 305 后的实机标定 SOP（必做）

换装 Gemini 305 + [urdf/相机壳3.0.3MF](urdf/相机壳3.0.3MF) 后，机械臂本体与夹爪标定（`home_pose.json` / `joint_offset.json` / `grasp.yaml`）可直接沿用，**重点完成以下 3 步新相机适配校验**：

1. **验证 Gemini 305 相机流、帧率与纯视觉（不连机械臂）**：
   ```bash
   python3 tools/enumerate_camera_modes.py
   python3 tools/camera_intrinsic_check.py
   python3 palm_follow_v14.py --no-arm
   ```
2. **复核轻量化载荷（`0.095 kg`）静态重力前馈（P3）**：
   因 Gemini 305 + 外壳比原 Gemini 2（`0.32 kg`）轻约 `0.225 kg`，建议跑一次分级重力校验确认 J2/J3/J4 不过补：
   ```bash
   python3 tools/p3_gravity_verify.py --pose home
   ```
3. **重新标定新支架 + Gemini 305 手眼外参（必做！）**：
   ```bash
   # 如需要更新 ArUco 基准观察点：
   python3 tools/hand_eye_calibration_auto.py
   # 联合求解 T_tcp_camera 与 6 轴零偏（残差 < 2.0 cm 合格）：
   python3 tools/handeye_joint_calib.py
   python3 tools/calib_diagnose.py
   ```
> 如需核对或重标 RobStride 机械零位/清错，使用内置工具 `python3 tools/rs_zero_calibrate.py`（`--clear-error` / `--set-zero`）。

---

## 六、日常运行指南（v14 视觉伺服 & v15 双向遥操作）

### 6.1 v14 手掌跟随 + 捏合抓取（`palm_follow_v14.py`）
```bash
source .venv/bin/activate

python3 palm_follow_v14.py --no-arm                              # 纯视觉演练
python3 palm_follow_v14.py --no-follow                           # 仅夹爪捏合
python3 palm_follow_v14.py --no-gripper --gain 1.5 --max-vel 0.15 # 低速仅跟随
python3 palm_follow_v14.py                                       # 真机完整模式（默认 distance=0.35m, 848×480@60fps）
```
常用参数：`--sim`、`--distance 0.25`（范围 `0.10 ~ 0.60 m`）、`--gain` / `--gain-x`、`--max-vel`、`--fps 30|60`、`--detector-backend auto|cpu|gpu`、`--no-gripper`、`--no-follow`、`--no-mujoco-sync`、`--headless`。
快捷键：**`g` / `Space`** 启停跟随（并清除 M6 锁存）；**`q` / `Esc` / `Ctrl+C`** 统一安全回零退出。

### 6.2 v15 MuJoCo Studio 3D 双向遥操作（`mujoco_teleop_v15.py`）
```bash
# 终端 1：启动 3D 窗口（支持 --urdf-variant soft_gripper|standard 及窗口内热切换；--web 浏览器模式）
./tools/launch_mujoco_twin.sh --native

# 终端 2：启动双向安全遥操作桥接
source .venv/bin/activate
python3 mujoco_teleop_v15.py --sim                 # 离线演练
python3 mujoco_teleop_v15.py --max-speed 20        # 真机低速安全双向联动
```

---

## 七、配置文件说明与按症状调参速查

| 配置文件 | 管理内容 | 关键参数 |
|---|---|---|
| [config/rs_mit.yaml](config/rs_mit.yaml) | CAN `can0`、7 路 RS06/RS00 参数、三级过温、MIT P0~P4 | `gravity_payload_mass_kg: 0.095`、`gravity_payload_com_link6_m: [0.055, 0.0, 0.075]` |
| [config/v14.yaml](config/v14.yaml) | 四层伺服、Gemini 305 深度保护与 RS 仿真动力学 | `pbvs.distance: 0.35`、`depth.min_valid_m: 0.07`、`camera.width: 848, height: 480, fps: 60` |
| [config/grasp.yaml](config/grasp.yaml) | 捏合像素范围、One Euro 滤波、J7 开合角 | `gripper.hold_while_moving: true` |

---

## 八、常见故障排查 FAQ

| 故障现象 | 原因与解决方法 |
|---|---|
| `can0` 不存在或 `Network is down` | 运行 `sudo ./tools/install_udev.sh` 拉起 SocketCAN 1Mbps |
| Gemini 305 无法打开或帧率不足 60fps | 1) 运行 `sudo ./tools/install_udev.sh` 并重插至 **USB 3.0 蓝色接口**；2) 确认已装 `pyorbbecsdk2` |
| 机械臂朝反方向跟随 / 偏差固定 | 当前仍为 0° 倾角名义占位外参：务必先运行 `tools/handeye_joint_calib.py` |
| 导入 `pinocchio` 时 `Segmentation fault` | 环境变量混入 `/opt/ros/*`，剔除后重跑 |
| 触发 `M6_FAULT` 原地锁死 | 碰撞/堵转/过力矩软保持（绝不掉臂）；排障确认安全后按 **`g`** 恢复 |

---

## 九、每次运行前安全 Checklist

- [ ] 机械臂底座已稳固夹紧，[urdf/相机壳3.0.3MF](urdf/相机壳3.0.3MF) 与 Gemini 305 固定牢靠、USB-C 线缆不干涉关节运动
- [ ] `can0` 已 `UP`（1Mbps），且仅运行单个控制进程
- [ ] 首次换装 Gemini 305 后，已完成 `p3_gravity_verify.py` 与 `handeye_joint_calib.py`（`calibrated: true`）
- [ ] `config/v14.yaml` 中工作空间与水平软墙 `workspace_max_radius ≤ 0.42 m` 符合当前桌面环境
- [ ] 改参后按 `--no-arm` → `--no-follow` → `--no-gripper --gain 1.5 --max-vel 0.15` → 完整模式逐级验收
- [ ] 退出一律等待程序完成「保持 → 缓慢回零到位 → 失能」，**严禁在屈肘抬臂状态强制断控失能**
