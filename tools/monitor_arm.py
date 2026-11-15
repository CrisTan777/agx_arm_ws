#!/usr/bin/env python3
"""实时监控 agx_arm 反馈（在 tmux 的 monitor 窗口运行）。

订阅:
  /feedback/joint_states  sensor_msgs/JointState
  /feedback/arm_status    diagnostic_msgs/DiagnosticArray
  /feedback/tcp_pose      geometry_msgs/PoseStamped
"""
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

CTRL_MODE = {
    0: 'STANDBY',
    1: 'CAN_CTRL',
    2: 'TEACHING',
    3: 'ETHERNET',
    4: 'WIFI',
    5: 'REMOTE',
    6: 'LINKAGE_TEACHING',
    7: 'OFFLINE_TRAJ',
}
ARM_STATUS = {
    0: 'NORMAL',
    1: 'EMERGENCY_STOP',
    2: 'NO_SOLUTION',
    3: 'SINGULARITY',
    4: 'TARGET_LIMIT',
    5: 'JOINT_COMM_ERROR',
    6: 'BRAKE_NOT_RELEASED',
    7: 'COLLISION',
    8: 'TEACH_OVERSPEED',
    9: 'JOINT_STATUS_ERR',
    10: 'OTHER_ERR',
}
TEACH_STATUS = {
    0: 'DISABLED',
    1: 'START_RECORDING',
    2: 'STOP_RECORDING',
    3: 'EXECUTE_TRAJECTORY',
    4: 'PAUSE',
    5: 'RESUME',
    6: 'TERMINATE',
    7: 'MOVE_TO_START',
}
MOTION_STATUS = {0: 'REACHED', 1: 'FAILED'}


def quat_to_rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class Monitor(Node):
    def __init__(self):
        super().__init__('agx_arm_monitor')
        self.joint_state = None
        self.joint_stamp = 0.0
        self.status = {}
        self.status_stamp = 0.0
        self.tcp_pose = None
        self.tcp_stamp = 0.0

        self.create_subscription(
            JointState, 'feedback/joint_states', self._on_joints, 10)
        self.create_subscription(
            DiagnosticArray, 'feedback/arm_status', self._on_status, 10)
        self.create_subscription(
            PoseStamped, 'feedback/tcp_pose', self._on_tcp, 10)
        self.create_timer(0.2, self._render)

    def _on_joints(self, msg):
        self.joint_state = msg
        self.joint_stamp = time.monotonic()

    def _on_status(self, msg):
        if msg.status:
            self.status = {kv.key: kv.value for kv in msg.status[0].values}
        self.status_stamp = time.monotonic()

    def _on_tcp(self, msg):
        self.tcp_pose = msg
        self.tcp_stamp = time.monotonic()

    def _render(self):
        now = time.monotonic()
        print('\033[H\033[2J', end='')
        print('=== agx_arm monitor ===  Ctrl+C 退出\n')

        if self.status:
            ctrl = int(self.status.get('ctrl_mode', -1))
            arm = int(self.status.get('arm_status', -1))
            teach = int(self.status.get('teach_status', -1))
            motion = int(self.status.get('motion_status', -1))
            err = self.status.get('err_status', '?')
            print(
                f"ctrl_mode : {CTRL_MODE.get(ctrl, ctrl)}\n"
                f"arm_status: {ARM_STATUS.get(arm, arm)}  (err_status={err})\n"
                f"teach     : {TEACH_STATUS.get(teach, teach)}\n"
                f"motion    : {MOTION_STATUS.get(motion, motion)}"
            )
        else:
            print('arm_status: 等待 /feedback/arm_status ...')

        print('\njoint       rad        deg')
        if self.joint_state is not None:
            for name, pos in zip(self.joint_state.name, self.joint_state.position):
                print(f'{name:<10} {pos: .4f}   {math.degrees(pos): .1f}')
            print(f'\nfeedback age: {now - self.joint_stamp:.2f}s')
        else:
            print('等待 /feedback/joint_states ... 检查 CAN 和 MoveIt 窗口')

        if self.tcp_pose is not None:
            p = self.tcp_pose.pose.position
            rpy = quat_to_rpy(self.tcp_pose.pose.orientation)
            print(
                f'\nTCP: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}  '
                f'rpy(deg)={math.degrees(rpy[0]):.1f}, '
                f'{math.degrees(rpy[1]):.1f}, {math.degrees(rpy[2]):.1f}'
            )
        else:
            print('\n等待 /feedback/tcp_pose ...')


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
