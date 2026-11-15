#!/usr/bin/env python3
"""把 Piper 系列从拖动示教模式切到 CAN 控制，并使能后保持当前姿态。

必须按官方手册顺序操作：
1. 在拖动示教模式下用示教器/手拖把机械臂摆到零位（各关节角≈0）；
2. 单击 J5/J6 之间的示教按键，LED 熄灭；
3. 运行本脚本：待机模式 -> CAN 控制模式 -> 使能 -> 保持当前姿态。

安全策略：
- 仍在示教模式时拒绝执行（固件会忽略 CAN 指令）；
- 关节角偏离零位超过 0.2 rad 时拒绝执行；
- 速度默认 10%；使能后以当前关节角为目标，不产生大幅运动。

用法：
    source tools/env.sh
    python3 tools/switch_to_can_control.py --robot piper_x --can can0
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


def connect_robot(robot_name: str, channel: str):
    cfg = create_agx_arm_config(robot=robot_name, interface="socketcan", channel=channel)
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()

    deadline = time.time() + 15
    while robot.get_firmware() is None:
        if time.time() >= deadline:
            robot.disconnect()
            raise RuntimeError("15s 内未收到固件信息，检查 CAN/电源")
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
    # 启用关节限位钳制，避免归零/保持指令越过安全限位
    robot.set_joint_limits_enabled(True)
    print(f"[OK] connected, firmware profile = {fw}")
    return robot


def wait_joint_angles(robot, timeout: float = 3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        ja = robot.get_joint_angles()
        if ja is not None:
            return list(ja.msg)
        time.sleep(0.05)
    raise RuntimeError("未收到关节角反馈")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="piper_x")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--speed", type=int, default=10, help="速度百分比 1-100")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--zero",
        action="store_true",
        help="使能后以当前速度低速运动到零位（而不是保持当前姿态）",
    )
    parser.add_argument("--zero-timeout", type=float, default=40.0)
    args = parser.parse_args()

    robot = connect_robot(args.robot, args.can)
    try:
        q0 = wait_joint_angles(robot)
        status = robot.get_arm_status()
        if status is None:
            raise RuntimeError("未收到机械臂状态")
        print(f"[INFO] ctrl_mode={status.msg.ctrl_mode} teach_status={status.msg.teach_status} "
              f"arm_status={status.msg.arm_status}")
        if int(status.msg.arm_status) != 0:
            raise RuntimeError(f"机械臂状态异常 arm_status={status.msg.arm_status}，停止操作")
        print(f"[INFO] current joints = {[round(q, 4) for q in q0]}")

        if int(status.msg.ctrl_mode) == TEACHING_MODE and int(status.msg.teach_status) == 0x01:
            print("[ABORT] 仍在拖动示教记录中，CAN 指令会被固件忽略。")
            print("        请单击 J5/J6 之间的示教按键，确认 LED 熄灭后重试本脚本。")
            return 2
        if int(status.msg.ctrl_mode) == TEACHING_MODE:
            print("[INFO] 示教记录已停止（teach_status=STOP_RECORDING），继续切换模式")

        max_dev = max(abs(q) for q in q0)
        if not args.zero and max_dev > 0.2:
            print(f"[ABORT] 当前最大关节角偏差 {max_dev:.3f} rad，未回到零位。")
            print("        请继续手拖归零（各关节角≈0）后再运行，或加 --zero 让程序归零。")
            return 3

        speed = max(1, min(100, args.speed))
        print("[STEP] 切换待机模式 ...")
        robot._send_msg(
            ArmMsgModeCtrl(
                ctrl_mode=STANDBY, move_mode=0x01, move_spd_rate_ctrl=speed
            )
        )
        time.sleep(0.5)

        print("[STEP] 切换 CAN 控制模式 ...")
        robot._send_msg(
            ArmMsgModeCtrl(
                ctrl_mode=CAN_CTRL, move_mode=0x01, move_spd_rate_ctrl=speed
            )
        )
        deadline = time.time() + args.timeout
        while time.time() < deadline:
            st = robot.get_arm_status()
            if st is not None and int(st.msg.ctrl_mode) == CAN_CTRL:
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("切换 CAN 控制模式失败（ctrl_mode 未变为 CAN_CTRL）")
        print("[OK] 已进入 CAN 控制模式")

        robot.set_speed_percent(speed)
        robot.set_motion_mode("j")
        print(f"[STEP] 使能电机（速度 {speed}%）...")

        deadline = time.time() + args.timeout
        while time.time() < deadline:
            if robot.enable():
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("使能超时")
        print("[OK] 电机已使能")

        if args.zero:
            target = [0.0] * len(q0)
            print(f"[STEP] 以 {speed}% 速度运动到零位 ...")
            robot.move_j(target)
            deadline = time.time() + args.zero_timeout
            last_print = 0.0
            while time.time() < deadline:
                ja = robot.get_joint_angles()
                if ja is not None:
                    q = list(ja.msg)
                    dev = max(abs(a - b) for a, b in zip(q, target))
                    if time.time() - last_print > 1.0:
                        print(f"  joints={[round(x, 3) for x in q]} max_dev={dev:.3f}")
                        last_print = time.time()
                    if dev < 0.03:
                        print("[OK] 已到达零位")
                        break
                time.sleep(0.05)
            else:
                print("[WARN] 归零超时，请检查状态；仍保持使能，可再次发送 move_j(0)")
        else:
            robot.move_j(q0)
            time.sleep(0.5)
            print("[OK] 已发送保持当前姿态指令")

        st = robot.get_arm_status()
        enable_list = robot.get_joints_enable_status_list()
        ja = robot.get_joint_angles()
        print(f"[RESULT] ctrl_mode={st.msg.ctrl_mode} arm_status={st.msg.arm_status} "
              f"motion_status={st.msg.motion_status}")
        print(f"[RESULT] joints_enabled={enable_list}")
        print(f"[RESULT] joints={[round(q, 4) for q in (list(ja.msg) if ja else [])]}")
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
