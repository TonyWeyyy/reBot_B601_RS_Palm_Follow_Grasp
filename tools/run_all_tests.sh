#!/usr/bin/env bash
# 一键运行全部 4 套离线验证单元测试（无真机 / 无相机即可执行）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}"

# 剔除 /opt/ros/* 避免与 Pinocchio 冲突
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="$(echo "${PYTHONPATH}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd: || true)"
fi
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  export LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd: || true)"
fi

# 选择主控 Python（优先使用项目内 .venv）
if [[ -n "${CONTROL_PYTHON:-}" ]]; then
  CTRL_PY="${CONTROL_PYTHON}"
elif [[ -x "${PROJECT_ROOT}/.venv/bin/python" ]]; then
  CTRL_PY="${PROJECT_ROOT}/.venv/bin/python"
else
  CTRL_PY="$(command -v python3)"
fi

echo "=================================================================="
echo "  使用主控 Python: ${CTRL_PY}"
echo "=================================================================="

echo ""
echo "[1/4] RobStride RS 底层控制 + P0~P4 + 配置红线与纯算法测试"
"${CTRL_PY}" tools/test_rs_controller.py

echo ""
echo "[2/4] v14 四层架构 + Gemini 305 深度保护链 + 数字孪生 + UDP 桥测试"
"${CTRL_PY}" tools/test_v14_architecture.py

echo ""
echo "[3/4] v15 双向遥操作安全隔离墙测试"
"${CTRL_PY}" tools/test_v15_teleop.py

# 选择 MuJoCo Studio Python
MJ_PY=""
if [[ -n "${MUJOCO_STUDIO_PYTHON:-}" && -x "${MUJOCO_STUDIO_PYTHON}" ]]; then
  MJ_PY="${MUJOCO_STUDIO_PYTHON}"
else
  for cand in \
    "${PROJECT_ROOT}/.venv_mujoco/bin/python" \
    "${PROJECT_ROOT}/.venv/bin/python" \
    "${CTRL_PY}" \
    "${HOME}/.venvs/mujoco-studio/bin/python"
  do
    if [[ -n "${cand}" && -x "${cand}" ]] && "${cand}" -c "import mujoco" >/dev/null 2>&1; then
      MJ_PY="${cand}"
      break
    fi
  done
fi

echo ""
if [[ -n "${MJ_PY}" ]]; then
  echo "[4/4] MuJoCo B601-RS + 相机壳3.0 STL + Gemini 305 双 URDF 场景编译测试 (${MJ_PY})"
  "${MJ_PY}" tools/test_mujoco_twin_rs.py
else
  echo "[4/4] 跳过 MuJoCo 场景测试（未检测到 mujoco>=4.0.1；运行 ./tools/setup_env.sh --mujoco-only 可安装）"
fi

echo ""
echo "=================================================================="
echo "  离线测试套件执行完毕！"
echo "=================================================================="
