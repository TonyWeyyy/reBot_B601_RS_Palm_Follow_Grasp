#!/usr/bin/env bash
# 一键启动 Seeed reBot Arm B601-RS + Gemini 2 MuJoCo Studio 3D 实时数字孪生
#
# 用法：
#   ./tools/launch_mujoco_twin.sh --native                           # 本地原生桌面 3D 窗口（默认柔性夹爪+支架 URDF）
#   ./tools/launch_mujoco_twin.sh --native --urdf-variant standard   # 切换为官方原版刚性夹爪 URDF（窗口内亦可热切换）
#   ./tools/launch_mujoco_twin.sh --native --urdf /path/to/arm.urdf  # 指定任意自定义 URDF
#   ./tools/launch_mujoco_twin.sh --web                              # 独立浏览器 Web 模式（http://localhost:8080/）
#   ./tools/launch_mujoco_twin.sh --web --open-browser               # 自动打开浏览器
#
# Python 解释器查找顺序（优先使用项目内独立环境，不依赖外部路径）：
#   1. $MUJOCO_STUDIO_PYTHON（手动指定）
#   2. <项目根目录>/.venv_mujoco/bin/python（推荐，由 ./tools/setup_env.sh 自动创建）
#   3. <项目根目录>/.venv/bin/python（若在统一 Python >= 3.11 环境中安装了 mujoco>=4.0.1）
#   4. 当前 PATH 中的 python3（若已安装 mujoco>=4.0.1）
#   5. $HOME/.venvs/mujoco-studio/bin/python（历史兼容回退）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

_has_mujoco_studio() {
  local py="$1"
  [[ -x "${py}" ]] && "${py}" -c "import mujoco; from mujoco import studio" >/dev/null 2>&1
}

STUDIO_PY=""
if [[ -n "${MUJOCO_STUDIO_PYTHON:-}" ]]; then
  STUDIO_PY="${MUJOCO_STUDIO_PYTHON}"
else
  for cand in \
    "${PROJECT_ROOT}/.venv_mujoco/bin/python" \
    "${PROJECT_ROOT}/.venv/bin/python" \
    "$(command -v python3 2>/dev/null || true)" \
    "${HOME}/.venvs/mujoco-studio/bin/python"
  do
    if [[ -n "${cand}" ]] && _has_mujoco_studio "${cand}"; then
      STUDIO_PY="${cand}"
      break
    fi
  done
fi

if [[ -z "${STUDIO_PY}" || ! -x "${STUDIO_PY}" ]]; then
  echo "[错误] 未找到可用的 MuJoCo Studio 4.0+ Python 环境。" >&2
  echo "请在项目根目录执行: ./tools/setup_env.sh --mujoco-only" >&2
  echo "或通过环境变量指定: MUJOCO_STUDIO_PYTHON=/path/to/python ./tools/launch_mujoco_twin.sh --native" >&2
  exit 1
fi

MODE="native"
PASS_ARGS=()
for arg in "$@"; do
  case "${arg}" in
    --native) MODE="native" ;;
    --web) MODE="web" ;;
    --sidecar) MODE="sidecar" ;;
    *) PASS_ARGS+=("${arg}") ;;
  esac
done

case "${MODE}" in
  native)
    exec "${STUDIO_PY}" "${PROJECT_ROOT}/mujoco_twin/run_twin.py" --native "${PASS_ARGS[@]}"
    ;;
  web)
    exec "${STUDIO_PY}" "${PROJECT_ROOT}/mujoco_twin/run_twin.py" --web "${PASS_ARGS[@]}"
    ;;
  sidecar)
    if [[ -z "${STUDIOD_SCRIPT:-}" || ! -f "${STUDIOD_SCRIPT}" ]]; then
      echo "[错误] --sidecar 模式需要通过环境变量 STUDIOD_SCRIPT 指定 studiod.py。" >&2
      echo "独立部署请直接使用: ./tools/launch_mujoco_twin.sh --native 或 --web" >&2
      exit 1
    fi
    exec "${STUDIO_PY}" "${STUDIOD_SCRIPT}" --workspace_dir="${PROJECT_ROOT}/mujoco_twin" "${PASS_ARGS[@]}"
    ;;
esac
