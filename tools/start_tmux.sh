#!/usr/bin/env bash
# 一键启动 tmux 工作台：MoveIt/RViz + 命令窗口 + 实时监控
#
# 用法:
#   bash tools/start_tmux.sh              # 启动并进入（夹爪 + DaBai DC1）
#   bash tools/start_tmux.sh --restart    # 重启整个工作台
#   bash tools/start_tmux.sh --no-enable  # 不自动使能机械臂
#   bash tools/start_tmux.sh --no-gripper # 无夹爪（同时关闭腕部相机）
#   bash tools/start_tmux.sh --no-camera  # 保留夹爪，不加载相机模型和画面
#   bash tools/start_tmux.sh --detach     # 只创建会话，不自动进入
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION=agx_arm
RESTART=0
DETACH=0
ENABLE=1
EFFECTOR=gripper
CAMERA_FLAG=""

usage() {
    sed -n '2,10p' "$0"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --restart) RESTART=1 ;;
        --detach) DETACH=1 ;;
        --no-enable) ENABLE=0 ;;
        --gripper) EFFECTOR=gripper ;;
        --no-gripper) EFFECTOR=none ;;
        --camera) CAMERA_FLAG="--camera" ;;
        --no-camera) CAMERA_FLAG="--no-camera" ;;
        --session) SESSION="${2:?--session 需要名称}"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数: $1"; usage; exit 1 ;;
    esac
    shift
done

if [[ "$EFFECTOR" == none && "$CAMERA_FLAG" == --camera ]]; then
    echo "腕部相机支架装在 AgxGripper 上，--camera 不能与 --no-gripper 一起使用。" >&2
    exit 1
fi

attach_session() {
    if [[ -n "${TMUX:-}" ]]; then
        tmux switch-client -t "$SESSION"
    else
        tmux attach -t "$SESSION"
    fi
}

if tmux has-session -t "$SESSION" 2>/dev/null; then
    if [[ $RESTART -eq 0 ]]; then
        echo "tmux 会话 '$SESSION' 已存在，直接进入（--restart 可重启）"
        if [[ $DETACH -eq 0 ]]; then
            attach_session
        fi
        exit 0
    fi
    echo "停止已有 tmux 会话 '$SESSION' ..."
    tmux kill-session -t "$SESSION" || true
fi

stop_existing() {
    if pgrep -f "start_single_agx_arm_moveit.launch.py" >/dev/null 2>&1; then
        echo "检测到已有 agx_arm MoveIt 进程，正在停止 ..."
        pkill -INT -f "start_single_agx_arm_moveit.launch.py" 2>/dev/null || true
        for _ in $(seq 1 50); do
            pgrep -f "start_single_agx_arm_moveit.launch.py" >/dev/null 2>&1 || break
            sleep 0.1
        done
        if pgrep -f "start_single_agx_arm_moveit.launch.py" >/dev/null 2>&1; then
            pkill -TERM -f "start_single_agx_arm_moveit.launch.py" 2>/dev/null || true
        fi
    fi
    pkill -INT -f "lib/agx_arm_ctrl/agx_arm_ctrl_single" 2>/dev/null || true

    # 清理可能残留的监控进程（tmux kill-session 后 Python 可能不退出）
    if pgrep -f "tools/monitor_arm.py" >/dev/null 2>&1; then
        pkill -INT -f "tools/monitor_arm.py" 2>/dev/null || true
        for _ in $(seq 1 10); do
            pgrep -f "tools/monitor_arm.py" >/dev/null 2>&1 || break
            sleep 0.1
        done
        pkill -TERM -f "tools/monitor_arm.py" 2>/dev/null || true
    fi
}

stop_existing

# 1) CAN
if ! ip link show can0 2>/dev/null | grep -q "UP"; then
    echo "can0 未启用，正在启用（可能需要 sudo 密码）..."
    bash "$WS/tools/can_up.sh"
fi

# 2) 预检：确认不是示教模式，并切到 CAN 控制模式（不使能、不运动）
echo "预检 CAN / 控制模式 ..."
set +e
python3 "$WS/tools/preflight_can_mode.py" --robot piper_x --can can0
PREFLIGHT_RC=$?
set -e

ENABLE_FLAG=""
case "$PREFLIGHT_RC" in
    0)
        [[ $ENABLE -eq 1 ]] && ENABLE_FLAG="--enable"
        ;;
    2)
        echo "预检需要人工处理：本次启动不会自动使能机械臂。"
        echo "处理示教模式后再执行: ros2 service call /enable_agx_arm std_srvs/srv/SetBool \"{data: true}\""
        ;;
    *)
        echo "预检失败，退出。"
        exit "$PREFLIGHT_RC"
        ;;
esac

# 3) tmux 工作台
tmux new-session -d -s "$SESSION" -n moveit -c "$WS"
tmux set-option -t "$SESSION" mouse on
tmux set-option -t "$SESSION" history-limit 20000

tmux send-keys -t "$SESSION:moveit" \
    "bash tools/bringup_moveit.sh $EFFECTOR $ENABLE_FLAG $CAMERA_FLAG" C-m

if [[ "$EFFECTOR" == gripper && "$CAMERA_FLAG" != --no-camera ]]; then
    tmux new-window -t "$SESSION" -n camera -c "$WS" -e "AGX_CAMERA_DEVICE=${AGX_CAMERA_DEVICE:-}"
    tmux send-keys -t "$SESSION:camera" \
        "source tools/env.sh && python3 tools/wrist_camera.py" C-m
fi

tmux new-window -t "$SESSION" -n shell -c "$WS"
tmux send-keys -t "$SESSION:shell" "source tools/env.sh" C-m
tmux send-keys -t "$SESSION:shell" "cat tools/CHEATSHEET.txt" C-m

tmux new-window -t "$SESSION" -n monitor -c "$WS"
tmux send-keys -t "$SESSION:monitor" \
    "source tools/env.sh && python3 tools/monitor_arm.py" C-m

tmux select-window -t "$SESSION:moveit"

echo "tmux 工作台已创建:"
echo "  moveit  - 驱动 + MoveIt + RViz（日志同时写入 /tmp/agx_arm_moveit.log）"
if [[ "$EFFECTOR" == gripper && "$CAMERA_FLAG" != --no-camera ]]; then
    echo "  camera  - DaBai DC1 彩色画面发布到 /wrist_camera/image_raw"
fi
echo "  shell   - 已 source tools/env.sh 的命令窗口"
echo "  monitor - 关节/状态实时监控"

if [[ $DETACH -eq 1 ]]; then
    echo "已按 --detach 创建，进入: tmux attach -t $SESSION"
elif [[ -t 1 ]]; then
    attach_session
else
    echo "非交互终端，进入: tmux attach -t $SESSION"
fi
