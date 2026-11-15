#!/usr/bin/env python3
"""AgxGripper 零点/行程标定。

前提：夹爪两指完全闭合、两指之间无异物；标定会以当前闭合位置作为 0 点。

用法:
    source tools/env.sh
    python3 tools/gripper_calibrate.py                 # 标定 + 张开/闭合验证
    python3 tools/gripper_calibrate.py --no-test       # 只标定
    python3 tools/gripper_calibrate.py --max-range 0.07
"""
import argparse
import time

from pyAgxArm import AgxArmFactory, create_agx_arm_config, resolve_firmware_profile


def connect(robot_name: str, channel: str):
    cfg = create_agx_arm_config(robot=robot_name, interface="socketcan", channel=channel)
    robot = AgxArmFactory.create_arm(cfg)
    gripper = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
    robot.connect()

    deadline = time.time() + 15
    while robot.get_firmware() is None:
        if time.time() >= deadline:
            robot.disconnect()
            raise RuntimeError("15s 内未收到机械臂固件信息")
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
    gripper = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
    robot.connect()
    return robot, gripper, fw


def status(gripper, timeout: float = 5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        gs = gripper.get_gripper_status()
        if gs is not None:
            return gs
        time.sleep(0.1)
    return None


def show(gs, label: str):
    foc = gs.msg.foc_status
    print(
        f"[{label}] width={gs.msg.value:.4f} m, force={gs.msg.force:.3f} N, "
        f"homing={foc.homing_status}, enabled={foc.driver_enable_status}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="AgxGripper zero calibration")
    parser.add_argument("--robot", default="piper_x")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--max-range", type=float, default=0.1, choices=[0.07, 0.1])
    parser.add_argument("--no-test", action="store_true")
    args = parser.parse_args()

    robot, gripper, fw = connect(args.robot, args.can)
    try:
        print(f"[OK] firmware={fw}, gripper initialized")
        gs = status(gripper)
        if gs is None:
            print("[FAIL] 未收到夹爪反馈")
            return 1
        show(gs, "before")
        if gs.msg.foc_status.homing_status:
            print("[INFO] 夹爪已标定，无需重复标定；仍继续设置行程并测试")
        if abs(gs.msg.value) > 0.01:
            print(f"[ABORT] 当前宽度 {gs.msg.value:.4f} m 不是闭合状态，先手动捏合再标定")
            return 2

        print("[STEP] 禁用夹爪驱动 ...")
        gripper.disable_gripper()
        time.sleep(0.5)

        print(f"[STEP] 设置最大行程 {args.max_range} m ...")
        ok_param = False
        for attempt in range(1, 4):
            if gripper.set_gripper_teaching_pendant_param(
                teaching_range_per=100,
                max_range_config=args.max_range,
                teaching_friction=1,
                timeout=2.0,
            ):
                ok_param = True
                break
            print(f"  第 {attempt} 次设置失败，重试")
            time.sleep(0.3)
        if not ok_param:
            print("[WARN] 设置最大行程失败，继续尝试标定")

        print("[STEP] 以当前闭合位置为零点进行标定 ...")
        ok_cal = False
        for attempt in range(1, 4):
            if gripper.calibrate_gripper(timeout=3.0):
                ok_cal = True
                break
            print(f"  第 {attempt} 次标定失败，重试")
            time.sleep(0.3)
        if not ok_cal:
            print("[FAIL] 标定失败：确认两指已完全闭合、无异物、夹爪供电正常")
            return 1

        time.sleep(0.5)
        gs = status(gripper)
        if gs is not None:
            show(gs, "after cal")
            if not gs.msg.foc_status.homing_status:
                print("[WARN] homing_status 仍为 false，检查夹爪固件/接线")

        if not args.no_test:
            print("[STEP] 张开到 0.05 m ...")
            gripper.move_gripper_m(value=0.05, force=1.0)
            time.sleep(1.5)
            gs = status(gripper, timeout=3.0)
            if gs is not None:
                show(gs, "open")

            print("[STEP] 闭合到 0.0 m ...")
            gripper.move_gripper_m(value=0.0, force=1.0)
            time.sleep(1.5)
            gs = status(gripper, timeout=3.0)
            if gs is not None:
                show(gs, "close")

        print("[DONE] 标定流程结束")
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
