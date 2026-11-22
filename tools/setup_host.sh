#!/usr/bin/env bash
# 首次环境配置：安装系统依赖 -> 启用 can0(1Mbps) -> 安装 pyAgxArm
# 需要 sudo 密码，请在普通终端执行：
#   bash tools/setup_host.sh
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

echo "==> [1/4] 安装系统依赖 (apt)"
sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  python3-opencv \
  can-utils \
  ethtool \
  ros-jazzy-topic-tools

echo "==> [2/4] 启用 CAN 接口 can0 @ 1000000 bps"
bash "$WS/src/agx_arm_ros/scripts/can_activate.sh" can0 1000000

echo "==> [3/4] 安装 Python 依赖与 pyAgxArm（editable 模式，便于二次开发）"
pip3 install --user --break-system-packages "python-can>=4.0" "typing-extensions>=3.7.4.3"
pip3 install --user --break-system-packages -e "$WS/third_party/pyAgxArm"

echo "==> [4/4] 自检"
python3 - <<'PY'
import can
from importlib.metadata import version
print("python-can:", can.__version__)
print("pyAgxArm  :", version("pyAgxArm"))
PY
ip -details link show can0 | grep -E "state|bitrate"

cat <<'EOF'

完成。以后每次重启电脑后，只需重新启用 CAN：
   bash tools/can_up.sh
EOF
