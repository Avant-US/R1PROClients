#!/usr/bin/env python3
"""
Monitor action commands and feedback simultaneously while run.py is running.
Records everything to a timestamped CSV + prints live summary to terminal.

Usage:
  # Terminal 1: run the model
  python3 run.py

  # Terminal 2: start monitoring
  python3 scripts/monitor.py

  # Press Ctrl+C to stop and see the summary + output file path.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import csv
import time
import sys
from pathlib import Path
from datetime import datetime
from collections import defaultdict

QOS_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)

QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)

TOPICS = {
    # --- action commands (published by run.py) ---
    # Use BEST_EFFORT so we can receive from both RELIABLE and BEST_EFFORT publishers
    "/motion_target/target_joint_state_arm_left":    ("cmd_left_arm",     JointState,   QOS_BEST_EFFORT),
    "/motion_target/target_joint_state_arm_right":   ("cmd_right_arm",    JointState,   QOS_BEST_EFFORT),
    "/motion_target/target_position_gripper_left":   ("cmd_left_gripper", JointState,   QOS_BEST_EFFORT),
    "/motion_target/target_position_gripper_right":  ("cmd_right_gripper",JointState,   QOS_BEST_EFFORT),
    "/motion_target/target_joint_state_torso":       ("cmd_torso",        JointState,   QOS_BEST_EFFORT),
    "/motion_target/target_speed_chassis":           ("cmd_chassis",      TwistStamped, QOS_BEST_EFFORT),
    # --- feedback (from robot) ---
    "/hdas/feedback_arm_left":      ("fb_left_arm",      JointState,   QOS_BEST_EFFORT),
    "/hdas/feedback_arm_right":     ("fb_right_arm",     JointState,   QOS_BEST_EFFORT),
    "/hdas/feedback_gripper_left":  ("fb_left_gripper",  JointState,   QOS_BEST_EFFORT),
    "/hdas/feedback_gripper_right": ("fb_right_gripper", JointState,   QOS_BEST_EFFORT),
    "/hdas/feedback_torso":         ("fb_torso",         JointState,   QOS_BEST_EFFORT),
    "/hdas/feedback_chassis":       ("fb_chassis",       JointState,   QOS_BEST_EFFORT),
}

CSV_HEADER = ["wall_time", "elapsed_s", "tag", "values"]


def extract_values(msg, tag: str) -> list[float]:
    if isinstance(msg, TwistStamped):
        return [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]
    if isinstance(msg, JointState):
        return list(msg.position)
    return []


class MonitorNode(Node):
    def __init__(self, csv_writer, start_time):
        super().__init__("action_monitor")
        self.csv_writer = csv_writer
        self.start_time = start_time
        self.counts = defaultdict(int)
        self.last_values = {}

        for topic, (tag, msg_type, qos) in TOPICS.items():
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=tag, tp=topic: self._on_msg(msg, t, tp),
                qos,
            )
            self.get_logger().info(f"Subscribed: {topic}  ->  [{tag}]")

        self.create_timer(2.0, self._print_summary)

    def _on_msg(self, msg, tag: str, topic: str):
        now = time.time()
        vals = extract_values(msg, tag)
        self.counts[tag] += 1
        self.last_values[tag] = vals
        self.csv_writer.writerow([
            f"{now:.6f}",
            f"{now - self.start_time:.4f}",
            tag,
            ",".join(f"{v:.6f}" for v in vals),
        ])

    def _print_summary(self):
        elapsed = time.time() - self.start_time
        lines = [f"\n--- {elapsed:.1f}s elapsed ---"]

        for topic, (tag, _, _) in TOPICS.items():
            cnt = self.counts.get(tag, 0)
            vals = self.last_values.get(tag)
            if vals is not None:
                short = " ".join(f"{v:+.4f}" for v in vals[:4])
                if len(vals) > 4:
                    short += " ..."
                lines.append(f"  {tag:22s}  msgs={cnt:5d}  last=[{short}]")
            else:
                lines.append(f"  {tag:22s}  msgs={cnt:5d}  (no data)")

        cmd_total = sum(c for t, c in self.counts.items() if t.startswith("cmd_"))
        fb_total = sum(c for t, c in self.counts.items() if t.startswith("fb_"))
        lines.append(f"  TOTAL  cmd={cmd_total}  fb={fb_total}")

        if cmd_total == 0 and elapsed > 5:
            lines.append("  ⚠ WARNING: No action commands received! Is run.py publishing?")

        print("\n".join(lines), flush=True)


def main():
    out_dir = Path("/tmp/monitor")
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / f"monitor_{stamp}.csv"

    print(f"Recording to: {csv_path}")
    print("Press Ctrl+C to stop.\n")

    rclpy.init()
    start_time = time.time()

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        node = MonitorNode(writer, start_time)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            elapsed = time.time() - start_time
            node.destroy_node()
            rclpy.shutdown()

    print(f"\n{'='*60}")
    print(f"Monitor stopped after {elapsed:.1f}s")
    print(f"Output: {csv_path}")
    print(f"\nTo analyze:")
    print(f"  python3 scripts/analyze_monitor.py {csv_path}")


if __name__ == "__main__":
    main()
