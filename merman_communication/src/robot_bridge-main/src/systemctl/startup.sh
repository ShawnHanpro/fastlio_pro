#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "请使用 sudo 运行：sudo bash $0"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SERVICE_SRC="${SCRIPT_DIR}/robot-bridge.service"
SERVICE_DST="/etc/systemd/system/robot-bridge.service"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"

if [[ ! -f "${SERVICE_SRC}" ]]; then
  echo "未找到 bridge service 模板: ${SERVICE_SRC}"
  exit 1
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "未找到 ROS2 环境文件: ${ROS_SETUP}"
  echo "如需指定路径，请设置 ROS_SETUP，例如：sudo ROS_SETUP=/opt/ros/humble/setup.bash bash $0"
  exit 1
fi

if [[ -n "${PYTHON_BIN:-}" ]]; then
  VENV_PY="${PYTHON_BIN}"
elif [[ -n "${VIRTUAL_ENV:-}" ]] && [[ -x "${VIRTUAL_ENV}/bin/python" ]]; then
  VENV_PY="${VIRTUAL_ENV}/bin/python"
elif [[ -x "${ROOT_DIR}/voice/env/bin/python" ]]; then
  VENV_PY="${ROOT_DIR}/voice/env/bin/python"
elif [[ -x "${ROOT_DIR}/env/bin/python" ]]; then
  VENV_PY="${ROOT_DIR}/env/bin/python"
else
  VENV_PY="/usr/bin/python3"
fi

if [[ ! -x "${VENV_PY}" ]]; then
  echo "Python 解释器不可执行: ${VENV_PY}"
  echo "如需指定解释器，请设置 PYTHON_BIN，例如：sudo PYTHON_BIN=/path/to/python bash $0"
  exit 1
fi

cp "${SERVICE_SRC}" "${SERVICE_DST}"

sed -i "s#^User=.*#User=root#" "${SERVICE_DST}"
sed -i "s#^Environment=PYTHONPATH=.*#Environment=PYTHONPATH=${ROOT_DIR}#" "${SERVICE_DST}"
sed -i "s#^WorkingDirectory=.*#WorkingDirectory=${ROOT_DIR}#" "${SERVICE_DST}"
sed -i "s#^ExecStart=.*#ExecStart=/usr/bin/env bash -lc 'source ${ROS_SETUP} && ${VENV_PY} -m robot_bridge'#" "${SERVICE_DST}"

systemctl daemon-reload
systemctl enable --now robot-bridge.service

echo "已完成：robot-bridge.service (User=root)"
echo "- 工作目录: ${ROOT_DIR}"
echo "- ROS2 环境: ${ROS_SETUP}"
echo "- Python解释器: ${VENV_PY}"
echo "查看状态：systemctl status robot-bridge"
echo "查看日志：journalctl -u robot-bridge -f"
