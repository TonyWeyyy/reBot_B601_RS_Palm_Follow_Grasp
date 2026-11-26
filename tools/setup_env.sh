#!/usr/bin/env bash
# ==============================================================================
# Seeed reBot Arm B601-RS + Orbbec Gemini 305 项目内独立环境一键搭建脚本
# ==============================================================================
# 会在项目根目录内创建两个完全独立的本地虚拟环境（不污染系统或外部路径）：
#   1. .venv/        : 主控制环境（Python 3.10+，优先从 third_party/wheels/ 安装
#                      motorbridge 与 pyorbbecsdk2，并安装 pin/mediapipe/opencv 等）
#   2. .venv_mujoco/ : MuJoCo Studio 4.0+ 3D 数字孪生环境（Python 3.12）
#
# 用法：
#   ./tools/setup_env.sh                 # 同时创建 .venv 和 .venv_mujoco
#   ./tools/setup_env.sh --control-only  # 仅创建主控环境 .venv
#   ./tools/setup_env.sh --mujoco-only   # 仅创建数字孪生环境 .venv_mujoco
#   ./tools/setup_env.sh --offline       # 严格离线模式（仅从 third_party/wheels* 安装）
#   ./tools/setup_env.sh --python /path/to/python3.10
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DO_CONTROL=1
DO_MUJOCO=1
OFFLINE=0
CONTROL_BASE_PY=""
MUJOCO_BASE_PY=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --control-only) DO_MUJOCO=0; shift ;;
    --mujoco-only) DO_CONTROL=0; shift ;;
    --offline) OFFLINE=1; shift ;;
    --python) CONTROL_BASE_PY="$2"; shift 2 ;;
    --mujoco-python) MUJOCO_BASE_PY="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,18p' "$0"
      exit 0
      ;;
    *)
      echo "[错误] 未知参数: $1" >&2
      exit 1
      ;;
  esac
done

# 剔除 /opt/ros/*，防止与 Pinocchio 发生 ABI 冲突
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="$(echo "${PYTHONPATH}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd: || true)"
fi
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  export LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd: || true)"
fi

WHEELS_DIR="${PROJECT_ROOT}/third_party/wheels"
WHEELS_MJ_DIR="${PROJECT_ROOT}/third_party/wheels_mujoco"

# ------------------------------------------------------------------------------
# 1. 主控制环境 (.venv)
# ------------------------------------------------------------------------------
if [[ "${DO_CONTROL}" -eq 1 ]]; then
  echo "=================================================================="
  echo "[1/2] 配置项目内主控制环境 -> ${PROJECT_ROOT}/.venv"
  echo "=================================================================="

  if [[ -z "${CONTROL_BASE_PY}" ]]; then
    if command -v python3.10 >/dev/null 2>&1; then
      CONTROL_BASE_PY="$(command -v python3.10)"
    elif command -v python3 >/dev/null 2>&1; then
      CONTROL_BASE_PY="$(command -v python3)"
    else
      echo "[错误] 未找到 python3，请先安装 python3 (>=3.10) 与 python3-venv。" >&2
      exit 1
    fi
  fi

  echo "  使用基础解释器: ${CONTROL_BASE_PY} ($("${CONTROL_BASE_PY}" --version))"
  if [[ ! -x "${PROJECT_ROOT}/.venv/bin/python" ]]; then
    if command -v uv >/dev/null 2>&1; then
      uv venv "${PROJECT_ROOT}/.venv" --python "${CONTROL_BASE_PY}" --seed
    else
      "${CONTROL_BASE_PY}" -m venv "${PROJECT_ROOT}/.venv"
    fi
  fi

  PIP_ARGS=(--find-links "${WHEELS_DIR}")
  if [[ "${OFFLINE}" -eq 1 ]]; then
    PIP_ARGS+=(--no-index)
  fi

  if command -v uv >/dev/null 2>&1; then
    uv pip install --python "${PROJECT_ROOT}/.venv/bin/python" "${PIP_ARGS[@]}" -r "${PROJECT_ROOT}/requirements.txt"
  else
    "${PROJECT_ROOT}/.venv/bin/pip" install --upgrade pip || true
    "${PROJECT_ROOT}/.venv/bin/pip" install "${PIP_ARGS[@]}" -r "${PROJECT_ROOT}/requirements.txt"
  fi
  echo "[完成] 主控制环境已就绪: ${PROJECT_ROOT}/.venv/bin/python"
fi

# ------------------------------------------------------------------------------
# 2. MuJoCo Studio 4.0+ 数字孪生独立环境 (.venv_mujoco)
# ------------------------------------------------------------------------------
if [[ "${DO_MUJOCO}" -eq 1 ]]; then
  echo ""
  echo "=================================================================="
  echo "[2/2] 配置项目内 MuJoCo Studio 4.0+ 环境 -> ${PROJECT_ROOT}/.venv_mujoco"
  echo "=================================================================="

  if [[ ! -x "${PROJECT_ROOT}/.venv_mujoco/bin/python" ]]; then
    if [[ -n "${MUJOCO_BASE_PY}" ]]; then
      "${MUJOCO_BASE_PY}" -m venv "${PROJECT_ROOT}/.venv_mujoco"
    elif command -v uv >/dev/null 2>&1; then
      uv venv "${PROJECT_ROOT}/.venv_mujoco" --python 3.12 --seed
    elif command -v python3.12 >/dev/null 2>&1; then
      python3.12 -m venv "${PROJECT_ROOT}/.venv_mujoco"
    elif command -v python3.11 >/dev/null 2>&1; then
      python3.11 -m venv "${PROJECT_ROOT}/.venv_mujoco"
    else
      echo "[错误] MuJoCo Studio 4.0+ 需要 Python >= 3.11（推荐 3.12）。" >&2
      echo "       请安装 uv (curl -LsSf https://astral.sh/uv/install.sh | sh) 或 python3.12 后重试，" >&2
      echo "       或仅运行 ./tools/setup_env.sh --control-only。" >&2
      exit 1
    fi
  fi

  MJ_PIP_ARGS=()
  if [[ -d "${WHEELS_MJ_DIR}" ]]; then
    MJ_PIP_ARGS+=(--find-links "${WHEELS_MJ_DIR}")
  fi
  if [[ "${OFFLINE}" -eq 1 ]]; then
    MJ_PIP_ARGS+=(--no-index)
  fi

  if command -v uv >/dev/null 2>&1; then
    uv pip install --python "${PROJECT_ROOT}/.venv_mujoco/bin/python" "${MJ_PIP_ARGS[@]}" -r "${PROJECT_ROOT}/requirements-mujoco.txt"
  else
    "${PROJECT_ROOT}/.venv_mujoco/bin/pip" install --upgrade pip || true
    "${PROJECT_ROOT}/.venv_mujoco/bin/pip" install "${MJ_PIP_ARGS[@]}" -r "${PROJECT_ROOT}/requirements-mujoco.txt"
  fi
  echo "[完成] MuJoCo Studio 环境已就绪: ${PROJECT_ROOT}/.venv_mujoco/bin/python"
fi

echo ""
echo "=================================================================="
echo "  全部环境配置完成！请运行以下自检命令确认："
echo "    ./.venv/bin/python tools/check_env.py"
echo "    ./tools/run_all_tests.sh"
echo "=================================================================="
