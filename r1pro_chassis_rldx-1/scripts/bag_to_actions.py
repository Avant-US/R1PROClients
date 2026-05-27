#!/usr/bin/env python3
"""Extract action commands from a rosbag (.mcap) into the format expected by visualizer/server.py."""

import argparse
import sys
from pathlib import Path

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    sys.exit("Missing rosbags library. Install with: pip install rosbags")


TOPIC_MAP = {
    "/motion_target/target_joint_state_arm_left": "left_arm",
    "/motion_target/target_joint_state_arm_right": "right_arm",
    "/motion_target/target_position_gripper_left": "left_gripper",
    "/motion_target/target_position_gripper_right": "right_gripper",
    "/motion_target/target_joint_state_torso": "torso",
    "/motion_target/target_speed_chassis": "chassis",
}


def extract_value(msg, topic: str) -> list[float]:
    if topic == "/motion_target/target_speed_chassis":
        return [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]
    return list(msg.position)


def main():
    parser = argparse.ArgumentParser(description="Convert rosbag action topics to visualizer txt")
    parser.add_argument("bag", type=Path, help="Path to rosbag directory")
    parser.add_argument("-o", "--output", type=Path, default=Path("/tmp/openpi_processed_actions.txt"),
                        help="Output file path (default: /tmp/openpi_processed_actions.txt)")
    parser.add_argument("--downsample", type=int, default=1,
                        help="Keep every Nth message per topic (default: 1 = keep all)")
    args = parser.parse_args()

    streams: dict[str, list[tuple[int, list[float]]]] = {v: [] for v in TOPIC_MAP.values()}

    with AnyReader([args.bag]) as reader:
        connections = [c for c in reader.connections if c.topic in TOPIC_MAP]
        for conn, timestamp, rawdata in reader.messages(connections=connections):
            key = TOPIC_MAP[conn.topic]
            msg = reader.deserialize(rawdata, conn.msgtype)
            val = extract_value(msg, conn.topic)
            streams[key].append((timestamp, val))

    for key in streams:
        streams[key].sort(key=lambda x: x[0])

    ref_key = max(streams, key=lambda k: len(streams[k]))
    ref_times = [t for t, _ in streams[ref_key]]
    n = len(ref_times)

    if n == 0:
        sys.exit("No action messages found in bag.")

    aligned: dict[str, list[list[float]]] = {}
    for key, entries in streams.items():
        if not entries:
            continue
        times = [t for t, _ in entries]
        vals = [v for _, v in entries]

        result = []
        j = 0
        for ref_t in ref_times:
            while j < len(times) - 1 and abs(times[j + 1] - ref_t) <= abs(times[j] - ref_t):
                j += 1
            result.append(vals[j])
        aligned[key] = result

    if args.downsample > 1:
        for key in aligned:
            aligned[key] = aligned[key][:: args.downsample]
        n = len(next(iter(aligned.values())))

    lines = []
    for key in ["left_arm", "right_arm", "left_gripper", "right_gripper", "torso", "chassis"]:
        if key not in aligned:
            continue
        rows = aligned[key]
        tensor_rows = ",\n        ".join(str(row) for row in rows)
        lines.append(f"[{key}]\ntensor([{tensor_rows}])\n")

    args.output.write_text("\n".join(lines))
    print(f"Wrote {n} frames to {args.output}")
    print(f"Sections: {[k for k in aligned]}")
    print(f"To visualize: python3 visualizer/server.py --actions {args.output}")


if __name__ == "__main__":
    main()
