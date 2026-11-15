#!/usr/bin/env python3
"""只读检测：确认 CAN 通信正常并打印固件版本、关节角（不会使能/不会让机械臂运动）。

用法:
    source /opt/ros/jazzy/setup.bash
    python3 tools/check_arm.py --robot piper_x --can can0
"""
import argparse
import time

from pyAgxArm import AgxArmFactory, create_agx_arm_config


def main() -> int:
    parser = argparse.ArgumentParser(description="AgileX arm read-only check")
    parser.add_argument("--robot", default="piper_x", help="piper / piper_x / piper_h / piper_l / nero")
    parser.add_argument("--can", default="can0", help="SocketCAN interface")
    args = parser.parse_args()

    cfg = create_agx_arm_config(robot=args.robot, interface="socketcan", channel=args.can)
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()

    deadline = time.time() + 15.0
    while robot.get_firmware() is None:
        if time.time() >= deadline:
            print(f"[FAIL] 15s 内未收到固件信息：检查机械臂电源、CAN 线、can0 是否已 1Mbps 启用")
            robot.disconnect()
            return 1
        time.sleep(1)

    fw = robot.get_firmware()
    print(f"[OK] firmware: {fw}")

    ja = robot.get_joint_angles()
    if ja is not None:
        print(f"[OK] joint_angles: {ja.msg}")
        print(f"     rate: {ja.hz} Hz, timestamp: {ja.timestamp}")
    else:
        print("[WARN] 未收到关节角反馈")

    robot.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
