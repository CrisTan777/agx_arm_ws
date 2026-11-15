#!/usr/bin/env python3
"""启动前预检（不使能、不运动）：
- 确认 CAN 通信和机械臂状态正常；
- 拒绝在拖动示教记录中启动；
- 必要时把机械臂从待机/示教停止状态切到 CAN 控制模式。

退出码:
  0 已处于 CAN 控制模式，可以启动
  2 需要人工处理（示教按键 / 先归零）
  1 其他错误
"""
import argparse
import time

from pyAgxArm import AgxArmFactory, create_agx_arm_config, resolve_firmware_profile
from pyAgxArm.protocols.can_protocol.msgs.piper.default.transmit.arm_mode_ctrl import (
    ArmMsgModeCtrl,
)

TEACHING_MODE = 0x02
CAN_CTRL = 0x01
STANDBY = 0x00
START_RECORDING = 0x01
STOP_RECORDING = 0x02


def connect(robot_name: str, channel: str):
    cfg = create_agx_arm_config(robot=robot_name, interface="socketcan", channel=channel)
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()

    deadline = time.time() + 10
    while robot.get_firmware() is None:
        if time.time() >= deadline:
            robot.disconnect()
            raise RuntimeError("10s 内未收到固件信息，检查 CAN/电源")
        time.sleep(0.2)

    fw = resolve_firmware_profile(robot_name, robot.get_firmware()["software_version"])
    robot.disconnect()

    cfg = create_agx_arm_config(
        robot=robot_name,
        firmeware_version=fw,
        interface="socketcan",
        channel=channel,
    )
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()
    return robot, fw


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="piper_x")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--speed", type=int, default=10, help="模式切换时附带的速度百分比")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    robot, fw = connect(args.robot, args.can)
    try:
        deadline = time.time() + 3
        status = None
        while time.time() < deadline:
            status = robot.get_arm_status()
            if status is not None:
                break
            time.sleep(0.1)
        if status is None:
            print("[FAIL] 未收到机械臂状态")
            return 1

        ctrl_mode = int(status.msg.ctrl_mode)
        teach_status = int(status.msg.teach_status)
        arm_status = int(status.msg.arm_status)
        print(
            f"[INFO] firmware={fw}, ctrl_mode={status.msg.ctrl_mode}, "
            f"teach_status={status.msg.teach_status}, arm_status={status.msg.arm_status}"
        )
        if arm_status != 0:
            print(f"[FAIL] 机械臂状态异常 arm_status={status.msg.arm_status}")
            return 1

        if ctrl_mode == TEACHING_MODE and teach_status == START_RECORDING:
            print("[ABORT] 正在拖动示教记录中，固件会忽略 CAN 指令。")
            print("        请单击 J5/J6 之间的示教按键让 LED 熄灭，再重新启动。")
            return 2

        if ctrl_mode == TEACHING_MODE and teach_status == STOP_RECORDING:
            ja = robot.get_joint_angles()
            q = list(ja.msg) if ja is not None else None
            if q is not None and max(abs(x) for x in q) > 0.2:
                print(f"[ABORT] 示教已停止但不在零位: {[round(x, 3) for x in q]}")
                print("        请先运行: python3 tools/switch_to_can_control.py --zero")
                return 2

        if ctrl_mode != CAN_CTRL:
            speed = max(1, min(100, args.speed))
            print("[STEP] 切换待机模式 ...")
            robot._send_msg(
                ArmMsgModeCtrl(ctrl_mode=STANDBY, move_mode=0x01, move_spd_rate_ctrl=speed)
            )
            time.sleep(0.4)
            print("[STEP] 切换 CAN 控制模式 ...")
            robot._send_msg(
                ArmMsgModeCtrl(ctrl_mode=CAN_CTRL, move_mode=0x01, move_spd_rate_ctrl=speed)
            )
            deadline = time.time() + args.timeout
            while time.time() < deadline:
                st = robot.get_arm_status()
                if st is not None and int(st.msg.ctrl_mode) == CAN_CTRL:
                    break
                time.sleep(0.05)
            else:
                print("[FAIL] 切换 CAN 控制模式失败")
                return 1

        st = robot.get_arm_status()
        print(f"[OK] ctrl_mode={st.msg.ctrl_mode}，可以启动 MoveIt")
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
