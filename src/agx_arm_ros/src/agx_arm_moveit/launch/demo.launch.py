import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

from _moveit_config_builder import (
    ALL_ARM_TYPES,
    ALL_EFFECTOR_TYPES,
    ALL_REVO2_TYPES,
    build_moveit_config,
)


def _build_ros2_controllers_file(arm_type, effector_type, revo2_type, namespace):
    """Build ros2_controllers config and return path to a temporary YAML file."""
    arm_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    if arm_type == "nero":
        arm_joints.append("joint7")

    cm_controllers = {
        "arm_controller": {
            "type": "joint_trajectory_controller/JointTrajectoryController",
        },
        "joint_state_broadcaster": {
            "type": "joint_state_broadcaster/JointStateBroadcaster",
        },
    }

    ns = namespace.strip("/")
    cm_node = f"/{ns}/controller_manager" if ns else "/controller_manager"

    config = {
        cm_node: {
            "ros__parameters": {"update_rate": 200, **cm_controllers},
        },
        (f"/{ns}/arm_controller" if ns else "/arm_controller"): {
            "ros__parameters": {
                "joints": arm_joints,
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            },
        },
    }

    if effector_type == "agx_gripper":
        cm_controllers["gripper_controller"] = {
            "type": "joint_trajectory_controller/JointTrajectoryController",
        }
        config[cm_node]["ros__parameters"].update(cm_controllers)
        config[(f"/{ns}/gripper_controller" if ns else "/gripper_controller")] = {
            "ros__parameters": {
                "joints": ["gripper"],
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            },
        }
    elif effector_type in ("revo2", "revo2_pro", "revo2_touch"):
        side = revo2_type
        ctrl_name = f"{side}_hand_controller"
        cm_controllers[ctrl_name] = {
            "type": "joint_trajectory_controller/JointTrajectoryController",
        }
        config[cm_node]["ros__parameters"].update(cm_controllers)
        config[(f"/{ns}/{ctrl_name}" if ns else f"/{ctrl_name}")] = {
            "ros__parameters": {
                "joints": [
                    f"{side}_thumb_metacarpal_joint",
                    f"{side}_thumb_proximal_joint",
                    f"{side}_index_proximal_joint",
                    f"{side}_middle_proximal_joint",
                    f"{side}_ring_proximal_joint",
                    f"{side}_pinky_proximal_joint",
                ],
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
            },
        }

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix="ros2_controllers_", delete=False
    )
    yaml.dump(config, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


def _build_namespaced_moveit_rviz_config(package_path, namespace, show_wrist_camera):
    """Generate the MoveIt RViz config for this namespace and hardware profile."""
    base_rviz = package_path / "config/moveit.rviz"
    config = yaml.safe_load(base_rviz.read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]
    ns = namespace.strip("/")
    move_group_ns = f"/{ns}" if ns else ""
    for display in displays:
        if display["Class"] == "moveit_rviz_plugin/MotionPlanning":
            display["Move Group Namespace"] = move_group_ns

    if show_wrist_camera:
        displays.append({
            "Class": "rviz_default_plugins/Image",
            "Name": "Wrist Camera",
            "Enabled": True,
            "Topic": {
                "Value": f"{move_group_ns}/wrist_camera/image_raw",
                "Reliability Policy": "Best Effort",
            },
            "Value": True,
        })

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".rviz", prefix="moveit_", delete=False
    )
    yaml.safe_dump(config, tmp, sort_keys=False)
    tmp.close()
    return tmp.name


def _build_moveit(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    arm_type = LaunchConfiguration("arm_type").perform(context)
    effector_type = LaunchConfiguration("effector_type").perform(context)
    revo2_type = LaunchConfiguration("revo2_type").perform(context)
    show_wrist_camera = (
        arm_type == "piper_x"
        and effector_type == "agx_gripper"
        and LaunchConfiguration("with_camera").perform(context) == "true"
    )
    moveit_config = build_moveit_config(context)
    package_path = moveit_config.package_path

    actions = []

    virtual_joints_launch = package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch))
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch/rsp.launch.py")
            )
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch/move_group.launch.py")
            )
        )
    )
    actions.append(
        Node(
            package="agx_arm_moveit",
            executable="agx_arm_control_gate",
            output="screen",
            parameters=[
                {
                    "status_topics": [
                        "arm_controller/follow_joint_trajectory/_action/status",
                        "gripper_controller/follow_joint_trajectory/_action/status",
                        "left_hand_controller/follow_joint_trajectory/_action/status",
                        "right_hand_controller/follow_joint_trajectory/_action/status",
                    ],
                    "gate_service_name": LaunchConfiguration("control_gate_service"),
                }
            ],
            condition=IfCondition(LaunchConfiguration("auto_control_gate")),
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch/moveit_rviz.launch.py")
            ),
            launch_arguments={
                "rviz_config": _build_namespaced_moveit_rviz_config(
                    package_path, namespace, show_wrist_camera
                ),
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    ros2_controllers_yaml = _build_ros2_controllers_file(
        arm_type, effector_type, revo2_type, namespace
    )
    actions.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                ros2_controllers_yaml,
            ],
            remappings=[("joint_states", LaunchConfiguration("control_topic"))],
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch/spawn_controllers.launch.py")
            )
        )
    )

    return [
        GroupAction(
            actions=[
                PushRosNamespace(namespace),
                SetRemap(src="/robot_description", dst="robot_description"),
                *actions,
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="ROS namespace for this arm instance (e.g. arm1).",
            ),
            DeclareLaunchArgument(
                "arm_type",
                default_value="piper",
                choices=ALL_ARM_TYPES,
                description="Arm type.",
            ),
            DeclareLaunchArgument(
                "effector_type",
                default_value="none",
                choices=ALL_EFFECTOR_TYPES,
                description="Effector type.",
            ),
            DeclareLaunchArgument(
                "revo2_type",
                default_value="left",
                choices=ALL_REVO2_TYPES,
                description="Revo2 / Revo2 Pro / Revo2 Touch hand side.",
            ),
            DeclareLaunchArgument(
                "with_camera",
                default_value="false",
                choices=["true", "false"],
                description="Attach the wrist camera assembly (stand + DaBai DC1). "
                "Only supported for piper_x with effector_type:=agx_gripper.",
            ),
            DeclareLaunchArgument(
                "tcp_offset",
                default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
                description="TCP offset [x, y, z, rx, ry, rz] in meters/radians.",
            ),
            DeclareLaunchArgument(
                "follow",
                default_value="false",
                choices=["true", "false"],
                description="Follow real arm state. "
                "true: move_group subscribes to feedback_topic; "
                "false: subscribes to control_topic (mock hardware).",
            ),
            DeclareBooleanLaunchArg(
                "db",
                default_value=False,
                description="By default, we do not start a database (it can be large)",
            ),
            DeclareBooleanLaunchArg(
                "debug",
                default_value=False,
                description="By default, we are not in debug mode",
            ),
            DeclareBooleanLaunchArg("use_rviz", default_value=True),
            DeclareBooleanLaunchArg(
                "auto_control_gate",
                default_value=False,
                description="Automatically gate /control commands during execute only.",
            ),
            DeclareLaunchArgument(
                "control_gate_service",
                default_value="control_enable",
                description="SetBool gate service for agx_arm_control_gate (maps to gate_service_name).",
            ),
            OpaqueFunction(function=_build_moveit),
        ]
    )
