#!/usr/bin/env bash
# 一键启动：真机驱动 + MoveIt2 + RViz
#
# 用法:
#   bash tools/bringup_moveit.sh                  # 无末端，不自动使能
#   bash tools/bringup_moveit.sh --enable         # 启动后自动使能（保持当前姿态）
#   bash tools/bringup_moveit.sh gripper --enable # 已装 AgxGripper + 自动使能
#   bash tools/bringup_moveit.sh gripper --no-camera  # 不加载腕部 DaBai DC1 模型
#
# 安全默认：10% 速度、auto_control_gate:=true（仅执行轨迹时开放控制）
# 装夹爪时默认加载腕部相机模型；没有相机时加 --no-camera。
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

EFFECTOR=none
AUTO_ENABLE=false
WITH_CAMERA=auto
for arg in "$@"; do
    case "$arg" in
        none) EFFECTOR=none ;;
        gripper|agx_gripper) EFFECTOR=agx_gripper ;;
        --enable) AUTO_ENABLE=true ;;
        --camera) WITH_CAMERA=true ;;
        --no-camera) WITH_CAMERA=false ;;
        -h|--help)
            sed -n '2,11p' "$0"
            exit 0
            ;;
        *)
            echo "未知参数: $arg（可用: none | gripper | --enable | --camera | --no-camera）" >&2
            exit 1
            ;;
    esac
done

if [[ "$WITH_CAMERA" == auto ]]; then
    if [[ "$EFFECTOR" == agx_gripper ]]; then WITH_CAMERA=true; else WITH_CAMERA=false; fi
fi

case "$EFFECTOR" in
    none) TCP_OFFSET='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]' ;;
    agx_gripper)
        # AgxGripper 指尖大约在法兰 Z 方向 142.5 mm 处
        TCP_OFFSET='[0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]'
        ;;
esac

if ! ip link show can0 2>/dev/null | grep -q "UP"; then
    echo "错误: can0 未启用，请先运行: bash $WS/tools/can_up.sh" >&2
    exit 1
fi

source "$WS/tools/env.sh"

if [[ "$AUTO_ENABLE" == true ]]; then
    echo "[INFO] 自动使能已开启：启动后机械臂会保持在当前姿态"
fi

ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
    can_port:=can0 \
    arm_type:=piper_x \
    effector_type:="$EFFECTOR" \
    with_camera:="$WITH_CAMERA" \
    tcp_offset:="$TCP_OFFSET" \
    auto_enable:="$AUTO_ENABLE" \
    speed_percent:=10 \
    auto_control_gate:=true 2>&1 | tee /tmp/agx_arm_moveit.log
