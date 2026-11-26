# 内置第三方硬件 SDK 与系统规则 (`third_party/`)

本目录随项目一起打包，使本项目在拷贝到任意目录或任意新主机后**无需依赖外部仓库**即可完成硬件驱动与 SDK 安装：

## 目录内容

1. **`wheels/` — 预打包硬件 SDK Wheel（Linux x86_64, Python 3.10）**
   - `motorbridge-0.5.5-cp310-cp310-manylinux_2_17_x86_64.whl`：RobStride / Damiao 统一电机控制 SDK（内含 Rust `libmotor_abi.so`，原生支持 SocketCAN `can0` 上的 RobStride RS06 / RS00 电机与 MIT 200Hz 反馈）。
   - `pyorbbecsdk2-2.1.2-cp310-cp310-manylinux_2_27_x86_64.whl`：奥比中光 Orbbec SDK v2 Python 封装（内含 `libOrbbecSDK.so.2.9.3`，原生支持 Gemini 305 60fps 与硬件 D2C 对齐）。
   - 运行 `./tools/setup_env.sh` 或 `pip install --find-links third_party/wheels -r requirements.txt` 时，pip 会**优先使用本目录中的本地 Wheel**；若目标主机为其他架构或不同 Python 小版本，pip 会自动回退从 PyPI 拉取对应平台版本。

2. **`udev/99-obsensor-libusb.rules` — 奥比中光全系列 USB 权限规则**
   - 包含 Orbbec Gemini 305 (`idVendor=2bc5`, `idProduct=0840/0841/0842`) 及全系 UVC/OpenNI USB 规则。
   - 通过 `sudo ./tools/install_udev.sh` 一键安装到 `/etc/udev/rules.d/`，配置 `video`/`dialout` 用户组并拉起 SocketCAN `can0` (1Mbps)。

## 其他内置静态资产（位于项目其他目录）

- `urdf/相机壳3.0.3MF`：Orbbec Gemini 305 腕部外壳 3D 打印源文件（0° 倾角前视，44×58×26 mm）；
- `urdf/Rebot_B601_RS/`：完整的 Seeed reBot Arm B601-RS 官方刚性夹爪 URDF（`ReBot_Arm_RS.urdf`）、改装柔性夹爪 URDF（`ReBot_Arm_RS_soft_gripper_cam.urdf`）、由 `相机壳3.0.3MF` 转换的 `gemini305_mount_v3.stl` 及全部视觉/碰撞 STL 网格；
- `urdf/Soft_Gripper_*.step`：柔性夹爪 3D 打印 CAD 源文件；
- `config/hand_landmarker.task`：MediaPipe 手部 21 关键点离线推理模型（7.5 MB，运行期零联网下载）。

## 如需打包 100% 断网全量 Wheel 仓库（可选）

```bash
pip download -d third_party/wheels -r requirements.txt
pip download -d third_party/wheels_mujoco --python-version 3.12 --only-binary=:all: -r requirements-mujoco.txt
```
