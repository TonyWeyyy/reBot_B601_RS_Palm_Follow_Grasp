# reBot Arm B601-RS + Orbbec Gemini 2 视觉伺服抓取与双向数字孪生系统（v14 + v15）

本工程为 **100% 独立自包含项目**：内置 B601-RS 双版本 URDF/STL 模型（**官方原版刚性夹爪** 与 **改装柔性夹爪+Gemini 2 支架**）、3D 打印 STEP 源文件、MediaPipe 离线手掌模型、RobStride `motorbridge` 与奥比中光 `pyorbbecsdk2` 硬件 SDK Wheel、Orbbec 全系 USB udev 规则、RS 独立机械零位标定工具以及一键环境搭建与自检脚本。**将本文件夹整体打包拷贝到任意机器、任意路径下，按本 README 操作即可独立完成安装、测试与运行，无需关联任何外部仓库。**

**一台 Gemini 2 相机、一次 MediaPipe 21 点检测、一条 `can0` 总线句柄**，即可同时实现：
- **v14 视觉伺服 + 捏合抓取（`palm_follow_v14.py`）**：手掌 3D 移动 → 机械臂保持固定姿态与期望距离（默认 `0.40 m`）实时跟随；拇指-食指捏合 → J7 夹爪同步开合；
- **v15 MuJoCo 双向数字孪生遥操作（`mujoco_teleop_v15.py` + `mujoco_twin/`）**：在本地原生 3D 窗口拖动关节滑块或 3D 目标球（实时 DLS IK）平滑安全驱动外部真机，支持**官方原版 URDF** 与 **改装柔性夹爪+相机支架 URDF** 一键热切换，同时 100Hz 实时镜像真机关节角、速度、力矩、温度与 3D 掌心，并支持一键回放历史高频 CSV 日志。

---

## 目录

1. [硬件清单与安全须知](#一硬件清单与安全须知)
2. [项目目录结构（全部内置依赖一览）](#二项目目录结构全部内置依赖一览)
3. [从零开始的环境配置与安装教程](#三从零开始的环境配置与安装教程)
4. [离线全量验证（无需硬件）](#四离线全量验证无需硬件)
5. [真机标定 SOP（新机/改支架时执行）](#五真机标定-sop新机改支架时执行)
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
| **末端夹爪** | 改装柔性夹爪 / 官方刚性夹爪 | `RS00`（ID `7`）驱动，共用 `can0` 单总线；数字孪生同时内置两版 URDF 支持随时切换 |
| **通信接口** | SocketCAN `can0` (1Mbps) | USB-CAN 适配器，`host_id = 0xFD`；通过 `sudo ./tools/install_udev.sh` 一键拉起 |
| **RGB-D 相机** | Orbbec Gemini 2（USB 3.0） | 理想工作距离 **`0.15 ~ 1.50 m`**，60fps 模式自动切至 `Binned Sparse Default` 并开启硬件 D2C 对齐；腕部相机+支架等效载荷 `0.32 kg` |
| **3D 打印件** | `urdf/*.step` | 内置 Gemini 2 相机支架（`D435_Gemini2_Mount_v1.step`）与柔性夹爪 CAD 源文件（`Soft_Gripper_Mount.step` / `Soft_Gripper_Finger.step`） |
| **推荐主机** | Ubuntu 22.04 / 20.04 x86_64 | 需至少 1 个 **USB 3.0 蓝色接口**（接 Gemini 2 跑 60fps）与 1 个 USB-CAN 接口 |

### 1.2 核心安全红线
> 1. **严禁非零位直接失能**：任何退出路径都必须走完「停止伺服 → 原地保持 → smoothstep 缓慢回零（重力前馈平滑淡出至 0）→ 到位失能」。
> 2. **严禁混用 B601-DM 配置**：B601-RS 的 `joint2` / `joint3` 为正区间（抬臂角度增大），关节轴向、力矩上限与 DM 完全不同。
> 3. **单一 CAN 主机**：全部 7 路电机（J1~J6 + J7 夹爪）共享同一个 `ReBotRSMITController`（`can0`），严禁同时启动第二个 `motorbridge` 进程。

---

## 二、项目目录结构（全部内置依赖一览）

```text
<项目根目录>/
├── palm_follow_v14.py             ★ v14.0 视觉伺服 + 捏合抓取主程序（支持真机 / --sim / --no-arm）
├── mujoco_teleop_v15.py           ★ v15.0 MuJoCo <-> B601-RS 双向遥操作桥接主程序
├── rs_controller.py               ★ B601-RS 独立 200Hz 底层控制器（直连 motorbridge + P0~P4）
├── rebot_adapter.py               ★ RS 机械臂适配层（Pinocchio 双模型隔离 + J7 单总线夹爪）
├── gemini2_camera.py              ★ Orbbec Gemini 2 SDK v2 采集层（60fps + HW D2C + 队列排空）
├── palm_detection.py              MediaPipe Tasks 21点检测 + 迟滞 ROI + LK 光流桥接 + 捏合提取
├── kalman_target.py               3D 恒速卡尔曼滤波（分轴 Q + 深度冻结期 XY 子空间更新）
├── servo_timing.py                时间校准 / 静止保持 / 伸展调度 / 软墙 / 振荡阻尼等纯算法
├── servo_diagnostics.py           非阻塞高频 CSV 诊断记录器
├── requirements.txt               主控制 Python 环境依赖清单（含 PyPI `pin`、SDK、OpenCV 等）
├── requirements-mujoco.txt        MuJoCo Studio 4.0+ 数字孪生环境依赖清单
│
├── third_party/                   ★ 内置第三方硬件 SDK 与系统规则（零外部仓库依赖）
│   ├── wheels/
│   │   ├── motorbridge-0.5.5-cp310-cp310-manylinux_2_17_x86_64.whl   (RobStride/DM 统一 SDK)
│   │   └── pyorbbecsdk2-2.1.2-cp310-cp310-manylinux_2_27_x86_64.whl  (奥比中光 SDK v2)
│   ├── udev/99-obsensor-libusb.rules
│   └── README.md
│
├── servo_v14/                     ★ v14/v15 四层解耦核心包
│   ├── config.py / types.py       不可变 dataclass 配置与强类型数据契约
│   ├── depth_guard.py             4 层深度保护链（适配 Gemini 2 0.15 ~ 1.50 m）
│   ├── estimator.py               层 1：统一世界系目标估计器（基于 200Hz 实测 q_act）
│   ├── state_machine.py           层 2：8 态显式有限状态机
│   ├── controller_pbvs.py         层 3：固定参考光轴世界系 PBVS 控制器
│   ├── safety_shield.py           层 4：不可绕过安全护盾（软/硬墙、奇异、限位缓冲、硬速红线）
│   ├── robot_backend.py           可插拔后端（HardwareRobotBackend + RS DigitalTwinBackend）
│   ├── pipeline.py                四层流水线编排器
│   ├── mujoco_bridge.py           双通道非阻塞本机 UDP 协议（状态 :19870 / 指令 :19871）
│   └── teleop_bridge.py           v15.0 遥操作安全控制器（5 道安全防线）
│
├── mujoco_twin/                   ★ MuJoCo Studio 4.0+ 3D 实时数字孪生工作区
│   ├── run_twin.py                独立启动器（--native / --web，支持 --urdf-variant 切换）
│   ├── scene_builder.py           B601-RS MjSpec 装配（双 URDF 变体 + Gemini 2 + 0.40m 光轴）
│   ├── sim_plugin.py              仿真插件（镜像 / 日志回放 / 关节滑块 / 3D 目标球 DLS IK）
│   ├── viewer_plugin.py           Dear ImGui 实时遥测、URDF 热切换与遥操作面板
│   └── messages.py                Sim <-> Viewer 强类型消息定义
│
├── urdf/                          ★ 内置 B601-RS 双 URDF 与 3D 打印 CAD 源文件
│   ├── Rebot_B601_RS/
│   │   ├── urdf/ReBot_Arm_RS.urdf                   （官方原版刚性夹爪 URDF）
│   │   ├── urdf/ReBot_Arm_RS_soft_gripper_cam.urdf  （改装柔性夹爪 + Gemini 2 支架 URDF）
│   │   └── meshes/                                  （全部视觉/碰撞 STL 网格）
│   ├── D435_Gemini2_Mount_v1.step
│   ├── Soft_Gripper_Finger.step
│   └── Soft_Gripper_Mount.step
│
├── config/                        ★ 仅 3 份核心 YAML + 已标定 JSON + 离线视觉模型
│   ├── rs_mit.yaml                ★ RS CAN/7路电机 + MIT P0~P4 统一配置
│   ├── v14.yaml                   ★ v14 视觉伺服、Gemini 2 深度保护与数字孪生参数
│   ├── grasp.yaml                 ★ J7 夹爪捏合映射与实测开合角（5.2° ~ 285.2°）
│   ├── home_pose.json             已标定工作 HOME 位姿（换机/改支架后用 teach_home.py 重录）
│   ├── hand_eye_calibration.json  已标定手眼外参 T_tcp_camera
│   ├── joint_offset.json          已标定 6 轴关节零偏
│   ├── viewpoint.json             手眼自动标定基准观察点
│   └── hand_landmarker.task       MediaPipe 手掌关键点离线推理模型（7.5 MB）
│
└── tools/                         安装、自检、标定、系统辨识与离线测试工具集
    ├── install_udev.sh            ★ 一键安装 Orbbec udev 规则、用户组与 SocketCAN can0 (1Mbps)
    ├── setup_env.sh               ★ 一键在项目内创建 .venv + .venv_mujoco 独立环境
    ├── check_env.py               ★ 一键自检依赖、内置资产、ROS 冲突与硬件权限
    ├── run_all_tests.sh           ★ 一键运行全部 4 套离线验证单元测试
    ├── launch_mujoco_twin.sh      ★ 一键启动 MuJoCo Studio 3D 窗口（自动选用 .venv_mujoco）
    ├── rs_zero_calibrate.py       ★ 独立 RobStride 机械零位标定 / 故障诊断清除 / 随动读角工具
    ├── test_rs_controller.py      RS 底层控制（P0~P4、RS06/RS00 红线、安全回零）单测
    ├── test_v14_architecture.py   v14 四层架构、Gemini 2 深度保护与闭环单测
    ├── test_v15_teleop.py         v15 双向遥操作安全防线单测
    ├── test_mujoco_twin_rs.py     MuJoCo RS 双 URDF（柔性夹爪+支架 / 官方刚性）编译自检
    ├── enumerate_camera_modes.py  枚举 Gemini 2 彩色/深度流档位
    ├── camera_intrinsic_check.py  校验 Gemini 2 内参、畸变与深度精度
    ├── benchmark_detector.py      MediaPipe 21点检测帧率/ROI 基准测试
    ├── teach_home.py              示教工作 HOME 位姿
    ├── gripper_j7_calibrate.py    标定 J7 夹爪闭合/全开角度
    ├── mit_feedback_probe.py      P0：核对 200Hz 反馈精度与重力符号
    ├── p3_gravity_verify.py       P3：分级静态重力前馈下垂校验
    ├── hand_eye_calibration_auto.py  手眼自动标定 / 基准点示教
    ├── handeye_joint_calib.py     ★ 推荐：手眼外参 + 6 关节零偏联合标定
    ├── calib_diagnose.py          标定旋转/平移一致性复核
    ├── joint_sysid.py             单轴正弦扫频系统辨识
    ├── analyze_servo_log.py       高频 CSV 伺服日志定量分析
    └── replay_digital_twin.py     离线数字孪生闭环回放与参数扫描
```

---

## 三、从零开始的环境配置与安装教程

将本项目拷贝到**任意路径**后，在终端进入项目根目录：

```bash
cd /path/to/rebot_B601_rs_palm_follow_grasp
```

采用 **项目内双虚拟环境解耦设计**（全部位于项目目录内，不污染系统）：
- **`.venv/`（主控环境，Python 3.10+）**：运行 `palm_follow_v14.py`、`mujoco_teleop_v15.py` 及全部标定工具；优先从 `third_party/wheels/` 安装 `motorbridge` 与 `pyorbbecsdk2`；
- **`.venv_mujoco/`（MuJoCo Studio 环境，Python 3.12）**：运行 3D 数字孪生窗口（`./tools/launch_mujoco_twin.sh`），与主控通过本机 UDP 零阻塞通信。

### 3.1 安装系统基础依赖（Ubuntu 20.04 / 22.04）
```bash
sudo apt-get update
sudo apt-get install -y \
    python3 python3-venv python3-pip \
    git curl build-essential pkg-config \
    libgl1 libglib2.0-0 libusb-1.0-0-dev \
    libxkbcommon-x11-0 libxcb-cursor0 \
    can-utils iproute2
```

> 建议安装 `uv`（自动下载 Python 3.12 秒建 `.venv_mujoco`）：
> ```bash
> curl -LsSf https://astral.sh/uv/install.sh | sh
> source "$HOME/.local/bin/env"
> ```

### 3.2 一键配置硬件权限与 SocketCAN `can0`
```bash
# 安装 Orbbec udev 规则 + 加入 video/dialout 组 + 拉起 SocketCAN can0 (1Mbps)
sudo ./tools/install_udev.sh

# 暂未接 USB-CAN 适配器时可附加 --no-can0：
# sudo ./tools/install_udev.sh --no-can0
```
完成后重新插拔 Gemini 2（插在 **USB 3.0 蓝色接口**），首次加组需注销重新登录，核对设备：
```bash
ip -details link show can0    # 应显示 state UP, bitrate 1000000
lsusb -d 2bc5:                # 应看到 Orbbec Gemini 2
```

### 3.3 一键搭建项目内独立 Python 环境
```bash
./tools/setup_env.sh
# 可选：--control-only / --mujoco-only / --offline
```
运行主控程序前激活项目虚拟环境：
```bash
source .venv/bin/activate
```

> ⚠️ **ROS 冲突提示**：若终端 `source` 过 `/opt/ros/*/setup.bash`，可能导致 `pinocchio` 导入段错误。运行前剔除 `/opt/ros`（`setup_env.sh` 与 `run_all_tests.sh` 已自动处理）：
> ```bash
> export PYTHONPATH=$(echo "${PYTHONPATH:-}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd:)
> export LD_LIBRARY_PATH=$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd:)
> ```

### 3.4 一键自检环境与内置资产
```bash
source .venv/bin/activate
python3 tools/check_env.py
```

---

## 四、离线全量验证（无需硬件）

在连接真机前，先一键跑完 4 套自动化测试（RS 底层控制、v14 架构、v15 遥操作、MuJoCo 双 URDF 编译）：

```bash
./tools/run_all_tests.sh
```

全部 `OK` 后，可离线启动一次双向数字孪生仿真熟悉界面与 URDF 切换：
```bash
# 终端 1：启动本地 3D 窗口（默认改装柔性夹爪+支架 URDF；加 --urdf-variant standard 切官方刚性版）
./tools/launch_mujoco_twin.sh --native

# 终端 2：启动 v15 离线数字孪生桥接
source .venv/bin/activate
python3 mujoco_teleop_v15.py --sim
```
在右侧面板即可随时在 **`Soft Gripper + Cam`**（改装柔性夹爪+Gemini 2支架）与 **`Standard Official`**（官方原版刚性夹爪）之间一键热切换，并体验 `Joint Sliders` 与 `3D IK Sphere`。

---

## 五、真机标定 SOP（新机/改支架时执行）

> 本仓库 `config/` 中已随附当前实机标定好的 `home_pose.json` / `hand_eye_calibration.json` / `joint_offset.json` / `viewpoint.json` / `grasp.yaml`。若在同一台已标定好的机械臂上使用，可直接跳至第六节；若**换新机械臂、重装支架或零位漂移**，按以下顺序重新标定：

1. **RobStride 机械零位确认 / 清错**：
   ```bash
   python3 tools/rs_zero_calibrate.py              # 安全只读随动读角（抬臂时 J2/J3 为正角）
   python3 tools/rs_zero_calibrate.py --clear-error # 故障诊断与清错
   python3 tools/rs_zero_calibrate.py --set-zero    # 完全收拢贴合限位支架后写入机械零位（需输入 YES）
   ```
2. **校验 Gemini 2 相机流与纯视觉**：
   ```bash
   python3 tools/enumerate_camera_modes.py
   python3 tools/camera_intrinsic_check.py
   python3 palm_follow_v14.py --no-arm
   ```
3. **验证 200Hz MIT 反馈与重力符号（P0 & P3）**：
   ```bash
   python3 tools/mit_feedback_probe.py --pose home
   python3 tools/p3_gravity_verify.py --pose home
   ```
4. **示教工作 HOME + 标定 J7 夹爪行程**：
   ```bash
   python3 tools/teach_home.py
   python3 tools/gripper_j7_calibrate.py
   ```
5. **手眼外参 + 6 关节零偏联合标定**（桌面固定 ArUco 标定板，残差 `< 2.0 cm` 合格）：
   ```bash
   python3 tools/hand_eye_calibration_auto.py
   python3 tools/handeye_joint_calib.py
   python3 tools/calib_diagnose.py
   ```

---

## 六、日常运行指南（v14 视觉伺服 & v15 双向遥操作）

### 6.1 v14 手掌跟随 + 捏合抓取（`palm_follow_v14.py`）
建议按“从弱到强”逐级验收：
```bash
source .venv/bin/activate

python3 palm_follow_v14.py --no-arm                              # 纯视觉演练
python3 palm_follow_v14.py --no-follow                           # 仅夹爪捏合
python3 palm_follow_v14.py --no-gripper --gain 1.5 --max-vel 0.15 # 低速仅跟随
python3 palm_follow_v14.py                                       # 真机完整模式（默认 distance=0.40m, 60fps）
```

**常用参数与快捷键**：
| 参数 / 快捷键 | 作用 |
|---|---|
| `--sim` | 真相机 + 离线 RS 数字孪生臂（不碰真机 CAN） |
| `--distance 0.35` | 覆盖期望跟随距离（`0.18 ~ 0.60 m`，默认 `0.40 m`） |
| `--gain 2.5 --gain-x 2.0` | 覆盖竖直/深度增益与水平偏航增益 |
| `--max-vel 0.40` | 覆盖末端线速度上限（硬红线 `≤ 0.85 m/s`） |
| `--no-gripper` / `--no-follow` | 禁用夹爪（自动打开防遮挡）/ 禁用手臂跟随 |
| `--no-mujoco-sync` / `--headless` | 关闭 UDP 同步 / 无窗口模式（SSH） |
| **`g` / `Space`** | 启动或暂停跟随（同时清除 M6 安全监督锁存） |
| **`q` / `Esc` / `Ctrl+C`** | 统一安全退出：**原地保持 → 缓慢回零到位 → 失能** |

### 6.2 v15 MuJoCo Studio 3D 双向数字孪生遥操作（`mujoco_teleop_v15.py`）
```bash
# 终端 1：启动 3D 窗口（默认柔性夹爪+支架版；加 --urdf-variant standard 切官方刚性版；加 --web 用浏览器）
./tools/launch_mujoco_twin.sh --native

# 终端 2：启动双向安全遥操作桥接
source .venv/bin/activate
python3 mujoco_teleop_v15.py --sim                 # 离线仿真演练
python3 mujoco_teleop_v15.py --max-speed 20        # 真机低速安全双向联动
```
面板操作：**选择 URDF 变体** → 切至 `Joint Sliders` 或 `3D IK Sphere`（双击橙金色球体 `Ctrl+右键` 平移）→ 点击 `Sync to Real Pose`（确保 `Align ≤ 8.0°`）→ 点击 `ARM Real Robot Follow` 解锁联动；随时可点 `DISARM / HOLD NOW` 或按 `Space`/`h` 急停保持，按 `q`/`Esc`/`Ctrl+C` 安全回零退出。无实时连接时可展开 `Hardware Log Replay` 回放 `logs/servo_v14_*.csv`。

---

## 七、配置文件说明与按症状调参速查

**参数优先级：命令行参数 > YAML 配置 > 代码默认值**。日常调参只需编辑以下 3 个文件：

| 配置文件 | 管理内容 | 关键切换项 |
|---|---|---|
| [config/rs_mit.yaml](config/rs_mit.yaml) | CAN `can0`、7 路 RS06/RS00 参数、三级过温、MIT P0~P4 开关与实测参数 | `vel_ff_enable` / `gravity_ff_enable` / `gain_sched_enable` / `supervisor_enable` 均可独立开关 |
| [config/v14.yaml](config/v14.yaml) | 四层伺服参数（估计器/FSM/PBVS/安全护盾）、Gemini 2 深度保护、RS 仿真动力学 | `pbvs.distance: 0.40`、`depth.min_valid_m: 0.15`、`safety.workspace_max_radius: 0.42` |
| [config/grasp.yaml](config/grasp.yaml) | 捏合像素范围、One Euro 滤波、J7 开合角、`hold_while_moving` | `gripper.hold_while_moving: true`（运动时冻结夹爪防抖） |

### 常见症状调参速查（`config/v14.yaml`）
| 症状 | 优先检查 / 调整项 | 调整方向 |
|---|---|---|
| 停手后机械臂仍有微小晃动 | 先用 `--no-gripper` 排除 J7 振动；复核 `p3_gravity_verify.py` | `fsm.deadband_in_m`: `0.020 → 0.024`；`depth.oe_min_cutoff`: `0.85 → 0.65` |
| 到位过冲、来回摆几下才停稳 | 检查 `pbvs.world_ff_gain` 与近端渐弱半径 | `pbvs.world_ff_gain`: `0.62 → 0.48`；`pbvs.ff_taper_dist`: `0.08 → 0.11` |
| 左右快速挥手时底座 J1 发冲 | 降低水平偏航增益或加大远端阻尼 | `pbvs.gain_x`: `2.20 → 1.70` 或 `rs_mit.yaml` 调大 `gain_sched_kd_far[0]` |
| 跟随滞后明显 | 确认 Gemini 2 跑在 60fps（USB 3.0）且 `rs_mit.yaml` 开启 `vel_ff_enable: true` | `pbvs.gain`: `2.75 → 2.95`；改善正面补光 |
| 快速挥手频繁丢手 | 室内运动模糊导致 MediaPipe 漏检 | 改善补光 + 确认 `camera.color_exposure_ms: 6.0` |

高频日志定量评估与数字孪生回放：
```bash
python3 tools/analyze_servo_log.py logs/servo_v14_<timestamp>.csv --json
python3 tools/replay_digital_twin.py logs/servo_v14_<timestamp>.csv
```

---

## 八、常见故障排查 FAQ

| 故障现象 | 原因与解决方法 |
|---|---|
| `can0` 不存在或 `Network is down` | 运行 `sudo ./tools/install_udev.sh` 拉起 SocketCAN 1Mbps（`ip link show can0` 应为 `UP`） |
| 电机无响应 / 报错锁死 | 1) 检查电源与单进程占用 `can0`；2) 运行 `python3 tools/rs_zero_calibrate.py --clear-error` 查故障并清错 |
| 导入 `pinocchio` 时 `Segmentation fault` | 环境变量混入 `/opt/ros/*`：执行 3.3 节剔除 `PYTHONPATH`/`LD_LIBRARY_PATH` 命令后重跑 |
| Gemini 2 无法打开或只有 30fps | 1) 运行 `sudo ./tools/install_udev.sh` 并重插相机至 **USB 3.0** 蓝色接口；2) 确认已装 `pyorbbecsdk2` 而非旧版 v1 |
| 机械臂朝反方向跟随 / 偏差固定 | 手眼标定与当前支架不匹配：重跑 `tools/handeye_joint_calib.py` |
| 运行中途触发 `M6_FAULT` 原地锁死 | 外力碰撞、限位顶死或力矩超阈值（安全软保持，不会掉臂）；排障确认安全后按 **`g`** 恢复 |
| `./tools/launch_mujoco_twin.sh` 找不到环境 | 执行 `./tools/setup_env.sh --mujoco-only` 创建项目内 `.venv_mujoco/` |
| 远程 SSH 下无法打开 3D 窗口 | 使用 `./tools/launch_mujoco_twin.sh --web --http-port 8080` 浏览器访问，主程序加 `--headless` |

---

## 九、每次运行前安全 Checklist

- [ ] 机械臂底座已稳固夹紧，相机支架、夹爪与 Gemini 2 固定牢靠、USB-C 线缆不干涉各关节运动
- [ ] `can0` 已 `UP`（1Mbps），且仅运行单个控制进程，无后台残留进程
- [ ] 换机或改动支架/零点后，已完成 `rs_zero_calibrate.py` → `teach_home.py` → `gripper_j7_calibrate.py` → `p3_gravity_verify.py` → `handeye_joint_calib.py`
- [ ] `config/v14.yaml` 中工作空间与水平软墙 `workspace_max_radius ≤ 0.42 m` 符合当前桌面环境
- [ ] 改参后按 `--no-arm` → `--no-follow` → `--no-gripper --gain 1.5 --max-vel 0.15` → 完整模式逐级验收
- [ ] 工作半径内无人员与易碎物；测试期间**盯紧机械臂实体、手边随时可按 `q` / `Esc` 或硬件电源开关**
- [ ] 退出一律等待程序完成「保持 → 缓慢回零到位 → 失能」，**严禁在屈肘抬臂状态强制断控失能**
