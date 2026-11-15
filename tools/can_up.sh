#!/usr/bin/env bash
# 每次开机后启用 CAN（Piper 系列固定 1Mbps）
set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-1000000}"

sudo ip link set "$IFACE" down 2>/dev/null || true
# 注意：gs_usb/candleLight 不支持 restart-ms，不能加自动 Bus-Off 恢复参数
sudo ip link set "$IFACE" type can bitrate "$BITRATE"
sudo ip link set "$IFACE" txqueuelen 1000
sudo ip link set "$IFACE" up

ip -details link show "$IFACE" | grep -E "state|bitrate"
