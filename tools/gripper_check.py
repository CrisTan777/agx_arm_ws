#!/usr/bin/env python3
"""AgxGripper 检测 / 开合测试（安装夹爪后使用）。

默认只读，不会动作；--open/--close/--width 才会驱动夹爪。

用法:
    source tools/env.sh
    python3 tools/gripper_check.py                # 只读状态
    python3 tools/gripper_check.py --open         # 张开到 0.1 m
    python3 tools/gripper_check.py --close        # 闭合
    python3 tools/gripper_check.py --width 0.05 --force 1.0
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


def wait_status(gripper, timeout: float = 10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        gs = gripper.get_gripper_status()
        if gs is not None:
            return gs
        time.sleep(0.1)
    return None


def print_status(gs):
    foc = gs.msg.foc_status
    print(f"[OK] width/angle = {gs.msg.value} (mode={gs.msg.mode}), force = {gs.msg.force} N")
    print(f"     homing_status={foc.homing_status}, driver_enable_status={foc.driver_enable_status}")
    print(
        "     faults: "
        f"voltage_low={foc.voltage_too_low}, motor_overheat={foc.motor_overheating}, "
        f"overcurrent={foc.driver_overcurrent}, driver_overheat={foc.driver_overheating}, "
        f"sensor={foc.sensor_status}, driver_error={foc.driver_error_status}"
    )
    print(f"     rate={gs.hz} Hz, timestamp={gs.timestamp}")


def main() -> int:
    parser = argparse.ArgumentParser(description="AgxGripper check / open-close test")
    parser.add_argument("--robot", default="piper_x")
    parser.add_argument("--can", default="can0")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--open", action="store_true", help="张开到 0.1 m")
    group.add_argument("--close", action="store_true", help="闭合到 0 m")
    group.add_argument("--width", type=float, help="目标宽度 m (0-0.1)")
    parser.add_argument("--force", type=float, default=1.0, help="夹持力 N (0-3)")
    args = parser.parse_args()

    robot, gripper, fw = connect(args.robot, args.can)
    try:
        print(f"[OK] firmware={fw}, gripper initialized")
        gs = wait_status(gripper)
        if gs is None:
            print("[FAIL] 10s 内未收到夹爪反馈：检查末端 XT30 线、夹爪是否上电、effector 是否为 agx_gripper")
            return 1
        print("[before]")
        print_status(gs)

        if not gs.msg.foc_status.homing_status:
            print("[WARN] homing_status=false，夹爪尚未标定，运动可能失败；先做夹爪置零/标定")

        target = None
        if args.open:
            target = 0.1
        elif args.close:
            target = 0.0
        elif args.width is not None:
            target = args.width

        if target is not None:
            if not (0.0 <= target <= 0.1):
                print("[FAIL] 目标宽度应在 0-0.1 m")
                return 1
            print(f"[STEP] 移动到宽度 {target} m，力 {args.force} N ...")
            gripper.move_gripper_m(value=target, force=args.force)
            time.sleep(1.5)
            print("[after]")
            gs = wait_status(gripper, timeout=3.0)
            if gs is not None:
                print_status(gs)
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
