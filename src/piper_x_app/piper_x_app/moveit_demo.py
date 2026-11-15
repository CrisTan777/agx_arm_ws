#!/usr/bin/env python3
"""Piper X MoveIt2 规划/执行示例。

默认只规划（plan only），不会让机械臂运动；加 --execute 才会执行轨迹。

用法:
    ros2 run piper_x_app moveit_demo home
    ros2 run piper_x_app moveit_demo home --execute
    ros2 run piper_x_app moveit_demo joint --values 0 0 0 0 0 0.3 --execute
    ros2 run piper_x_app moveit_demo pose --xyz 0.2 0.0 0.35 --rpy 0 90 0
"""
import argparse
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def quat_from_rpy(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def joint_constraints(target, tolerance=0.01) -> Constraints:
    c = Constraints()
    for name, value in zip(JOINT_NAMES, target):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = float(value)
        jc.tolerance_above = tolerance
        jc.tolerance_below = tolerance
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    return c


def gripper_constraints(width, tolerance=0.005) -> Constraints:
    c = Constraints()
    jc = JointConstraint()
    jc.joint_name = 'gripper'
    jc.position = float(width)
    jc.tolerance_above = tolerance
    jc.tolerance_below = tolerance
    jc.weight = 1.0
    c.joint_constraints.append(jc)
    return c


def pose_constraints(
    frame: str,
    link: str,
    xyz,
    rpy,
    position_tolerance=0.01,
    orientation_tolerance=0.05,
) -> Constraints:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
    qx, qy, qz, qw = quat_from_rpy(*map(float, rpy))
    pose.orientation.x, pose.orientation.y = qx, qy
    pose.orientation.z, pose.orientation.w = qz, qw

    c = Constraints()

    pc = PositionConstraint()
    pc.header.frame_id = frame
    pc.link_name = link
    pc.target_point_offset.x = 0.0
    pc.target_point_offset.y = 0.0
    pc.target_point_offset.z = 0.0
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [position_tolerance]
    pc.constraint_region.primitives.append(sphere)
    pc.constraint_region.primitive_poses.append(pose)
    pc.weight = 1.0
    c.position_constraints.append(pc)

    oc = OrientationConstraint()
    oc.header.frame_id = frame
    oc.link_name = link
    oc.orientation = pose.orientation
    oc.absolute_x_axis_tolerance = orientation_tolerance
    oc.absolute_y_axis_tolerance = orientation_tolerance
    oc.absolute_z_axis_tolerance = orientation_tolerance
    oc.weight = 1.0
    c.orientation_constraints.append(oc)

    return c


class MoveItDemo(Node):
    def __init__(self):
        super().__init__('piper_x_moveit_demo')
        self.declare_parameter('group_name', 'arm')
        self.declare_parameter('planning_frame', 'world')
        self.declare_parameter('eef_link', 'tcp_link')
        self.declare_parameter('action_name', '/move_action')
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('acceleration_scaling', 0.1)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('planning_attempts', 5)

        action_name = self.get_parameter('action_name').value
        self.client = ActionClient(self, MoveGroup, action_name)

    def plan_and_execute(self, constraints: Constraints, execute: bool) -> bool:
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveIt /move_action 不可用，先启动 MoveIt')
            return False

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.get_parameter('group_name').value
        req.num_planning_attempts = int(self.get_parameter('planning_attempts').value)
        req.allowed_planning_time = float(self.get_parameter('planning_time').value)
        req.max_velocity_scaling_factor = float(
            self.get_parameter('velocity_scaling').value
        )
        req.max_acceleration_scaling_factor = float(
            self.get_parameter('acceleration_scaling').value
        )
        req.goal_constraints.append(constraints)

        goal.planning_options.plan_only = not execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        self.get_logger().info(
            f'发送 {"规划+执行" if execute else "仅规划"} 请求 '
            f'(group={req.group_name}, vel={req.max_velocity_scaling_factor}, '
            f'acc={req.max_acceleration_scaling_factor})'
        )

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('MoveIt 拒绝了目标')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        traj = result.planned_trajectory.joint_trajectory
        if traj.points:
            last = traj.points[-1]
            end_state = {
                name: round(pos, 4)
                for name, pos in zip(traj.joint_names, last.positions)
            }
            self.get_logger().info(f'轨迹终点: {end_state}')

        code = result.error_code.val
        if code != 1:
            self.get_logger().error(f'规划/执行失败，MoveItErrorCodes={code}')
            return False

        points = len(result.planned_trajectory.joint_trajectory.points)
        self.get_logger().info(
            f'成功，轨迹点数={points}，'
            f'{"已执行" if execute else "未执行（plan only）"}'
        )
        return True


def _add_common_args(parser, suppress_defaults=False):
    default = argparse.SUPPRESS if suppress_defaults else False
    parser.add_argument(
        '--execute',
        action='store_true',
        default=default,
        help='执行轨迹；不加此参数只规划',
    )
    parser.add_argument(
        '--vel',
        type=float,
        default=argparse.SUPPRESS if suppress_defaults else 0.1,
        help='速度缩放 0-1',
    )
    parser.add_argument(
        '--acc',
        type=float,
        default=argparse.SUPPRESS if suppress_defaults else 0.1,
        help='加速度缩放 0-1',
    )


def build_parser():
    parser = argparse.ArgumentParser(description='Piper X MoveIt2 planning demo')
    _add_common_args(parser)

    sub = parser.add_subparsers(dest='command', required=True)

    sub.add_parser('home', help='回到零位（所有关节 0）', parents=[])
    _add_common_args(sub.choices['home'], suppress_defaults=True)

    p_joint = sub.add_parser('joint', help='关节空间目标')
    _add_common_args(p_joint, suppress_defaults=True)
    p_joint.add_argument(
        '--values',
        nargs=6,
        type=float,
        required=True,
        metavar=('J1', 'J2', 'J3', 'J4', 'J5', 'J6'),
        help='六个关节角，单位 rad',
    )

    p_gripper = sub.add_parser('gripper', help='夹爪宽度目标（需 effector_type:=agx_gripper）')
    _add_common_args(p_gripper, suppress_defaults=True)
    p_gripper.add_argument(
        '--width',
        type=float,
        default=0.05,
        help='夹爪开度，单位 m（0-0.1）',
    )

    p_pose = sub.add_parser('pose', help='笛卡尔位姿目标（tcp_link）')
    _add_common_args(p_pose, suppress_defaults=True)
    p_pose.add_argument('--xyz', nargs=3, type=float, required=True, metavar=('X', 'Y', 'Z'))
    p_pose.add_argument(
        '--rpy',
        nargs=3,
        type=float,
        default=(0.0, 0.0, 0.0),
        metavar=('R', 'P', 'Y'),
        help='欧拉角，单位度',
    )
    return parser


def main():
    args = build_parser().parse_args()

    rclpy.init()
    node = MoveItDemo()
    node.set_parameters([
        rclpy.parameter.Parameter('velocity_scaling', value=args.vel),
        rclpy.parameter.Parameter('acceleration_scaling', value=args.acc),
    ])

    try:
        if args.command == 'home':
            constraints = joint_constraints([0.0] * 6)
        elif args.command == 'joint':
            constraints = joint_constraints(args.values)
        elif args.command == 'gripper':
            if not 0.0 <= args.width <= 0.1:
                raise SystemExit('夹爪宽度必须在 0-0.1 m')
            node.set_parameters([
                rclpy.parameter.Parameter('group_name', value='gripper'),
            ])
            constraints = gripper_constraints(args.width)
        elif args.command == 'pose':
            frame = node.get_parameter('planning_frame').value
            link = node.get_parameter('eef_link').value
            rpy_rad = [math.radians(v) for v in args.rpy]
            constraints = pose_constraints(frame, link, args.xyz, rpy_rad)
        else:
            raise RuntimeError(f'未知命令: {args.command}')

        ok = node.plan_and_execute(constraints, execute=args.execute)
        raise SystemExit(0 if ok else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
