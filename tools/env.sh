#!/usr/bin/env bash
# 进入 agx_arm 工作空间环境（source 使用）：
#   source tools/env.sh
#
# 使用独立 ROS_DOMAIN_ID，避免与同机 nexArm_ws（默认域 0）话题串扰。
export ROS_DOMAIN_ID=42

# ROS 的 setup.bash 引用了未定义变量，调用方若开了 set -u 会报错；
# 这里临时关闭 nounset，source 完再恢复。
case $- in
    *u*) _agx_had_nounset=1 ;;
    *)   _agx_had_nounset=0 ;;
esac
set +u

source /opt/ros/jazzy/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/install/setup.bash"

[[ $_agx_had_nounset -eq 1 ]] && set -u
unset _agx_had_nounset
