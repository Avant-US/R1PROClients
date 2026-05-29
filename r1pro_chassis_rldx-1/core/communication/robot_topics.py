from typing import Dict, Literal
from dataclasses import dataclass, field
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from geometry_msgs.msg import PoseStamped, TwistStamped

@dataclass(frozen=True)
class Topic:
    channel: str
    msg_type: CompressedImage | JointState | PoseStamped | TwistStamped

@dataclass
class RobotTopicsConfig:
    state: Dict[str, Topic] = field(
        default_factory=lambda: {
            "left_arm": Topic("/hdas/feedback_arm_left", JointState),
            "right_arm": Topic("/hdas/feedback_arm_right", JointState),
            "torso": Topic("/hdas/feedback_torso", JointState),
            "chassis": Topic("/hdas/feedback_chassis", JointState),
            "left_ee_pose": Topic("/motion_control/pose_ee_arm_left", PoseStamped),
            "right_ee_pose": Topic("/motion_control/pose_ee_arm_right", PoseStamped),
            "left_gripper": Topic("/hdas/feedback_gripper_left", JointState),
            "right_gripper": Topic("/hdas/feedback_gripper_right", JointState),
        }
    )

    images: Dict[str, Topic] = field(
        default_factory=lambda: {
            "head_rgb": Topic("/hdas/camera_head/left_raw/image_raw_color/compressed", CompressedImage),
            "left_wrist_rgb": Topic("/hdas/camera_wrist_left/color/image_raw/compressed", CompressedImage),
            "right_wrist_rgb": Topic("/hdas/camera_wrist_right/color/image_raw/compressed", CompressedImage),
        }
    )

    action: Dict[str, Topic] = field(
        default_factory=lambda: {
            "left_arm": Topic("/motion_target/target_joint_state_arm_left", JointState),
            "right_arm": Topic("/motion_target/target_joint_state_arm_right", JointState),
            "torso": Topic("/motion_target/target_joint_state_torso", JointState),
            "chassis": Topic("/motion_target/target_speed_chassis", TwistStamped),
            "left_ee_pose": Topic("/motion_target/target_pose_arm_left", PoseStamped),
            "right_ee_pose": Topic("/motion_target/target_pose_arm_right", PoseStamped),
            "left_gripper": Topic("/motion_target/target_position_gripper_left", JointState),
            "right_gripper": Topic("/motion_target/target_position_gripper_right", JointState),
        }
    )

    qos: Dict[str, QoSProfile] = field(
        default_factory=lambda: {
            "sub": QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE
            ),
            "pub": QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE
            ),
        }
    )

    # 为了让 gather_obs 能按 buffer 索引回溯历史视频帧（rldx1 等多帧模型按
    # 视频帧 index 取 delta_indices=[-6,-4,-2,0]，和训练采集时一致），
    # 默认保留 30 帧。按相机 30Hz 算 ~1s，15Hz 算 ~2s 历史。
    # 单帧 raw RGB ~2-3MB，三路 30 帧约 240MB 内存。
    camera_deque_length: int = 30
    state_deque_length: int = 80  # >400 Hz, for 0.2s
