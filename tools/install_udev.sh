#!/usr/bin/env bash
# 一键安装 B601-RS 项目的系统设备权限与 CAN 总线：
#   1. 将内置 third_party/udev/99-obsensor-libusb.rules 安装到 /etc/udev/rules.d/
#   2. 将当前用户加入 video / dialout 用户组
#   3. 重新加载 udev 规则
#   4. 默认尝试配置并拉起 SocketCAN can0 (1Mbps)（可用 --no-can0 跳过）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RULES_SRC="${PROJECT_ROOT}/third_party/udev/99-obsensor-libusb.rules"
RULES_DST="/etc/udev/rules.d/99-obsensor-libusb.rules"

SETUP_CAN0=1
CAN_IF="can0"
for arg in "$@"; do
  case "${arg}" in
    --no-can0) SETUP_CAN0=0 ;;
    --can0) SETUP_CAN0=1 ;;
    -h|--help)
      echo "用法: $0 [--no-can0]"
      echo "  安装 Orbbec Gemini 2 udev 规则、配置 video/dialout 权限，并拉起 SocketCAN can0 (1Mbps)。"
      echo "  --no-can0  跳过 SocketCAN can0 配置（例如暂未接 USB-CAN 适配器时）"
      exit 0
      ;;
    *)
      echo "[错误] 未知参数: ${arg}" >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "${RULES_SRC}" ]]; then
  echo "[错误] 未找到内置 udev 规则文件: ${RULES_SRC}" >&2
  exit 1
fi

TARGET_USER="${SUDO_USER:-$USER}"
SUDO=""
if [[ "${EUID}" -ne 0 ]]; then
  SUDO="sudo"
fi

echo "[1/3] 安装 Orbbec USB udev 规则 -> ${RULES_DST}"
${SUDO} install -m 0644 "${RULES_SRC}" "${RULES_DST}"
${SUDO} udevadm control --reload-rules
${SUDO} udevadm trigger

echo "[2/3] 将用户 '${TARGET_USER}' 加入 video 与 dialout 用户组"
${SUDO} usermod -aG video,dialout "${TARGET_USER}"

if [[ "${SETUP_CAN0}" -eq 1 ]]; then
  echo "[3/3] 配置 SocketCAN ${CAN_IF} (1Mbps)..."
  if ip link show "${CAN_IF}" >/dev/null 2>&1; then
    ${SUDO} ip link set "${CAN_IF}" down 2>/dev/null || true
    ${SUDO} ip link set "${CAN_IF}" type can bitrate 1000000
    ${SUDO} ip link set "${CAN_IF}" up
    ip -details link show "${CAN_IF}"
  else
    echo "  [提示] 当前未检测到网络接口 ${CAN_IF}（请确认 USB-CAN 适配器已插入后再重新运行本脚本）"
  fi
else
  echo "[3/3] 已跳过 SocketCAN 配置（--no-can0）"
fi

echo ""
echo "=================================================================="
echo "  硬件权限安装完成！"
echo "  1. 请【重新插拔】Orbbec Gemini 2 USB 3.0 线；"
echo "  2. 如首次加入 video/dialout 组，请【注销重新登录】生效；"
echo "  3. 主机重启后如 can0 未自动拉起，重新执行 sudo ./tools/install_udev.sh 即可。"
echo "=================================================================="
