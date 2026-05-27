#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ast
import json
import mimetypes
import re
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET


DEFAULT_URDF_PATH = Path("/home/nvidia/zwy_WS/r1pro_chassis_rldx-1/assets/urdf/r1pro/r1_pro_with_gripper.urdf")
DEFAULT_ACTIONS_PATH = Path("/tmp/openpi_processed_actions.txt")
STATIC_DIR = Path(__file__).resolve().parent / "static"

SECTION_PATTERN = re.compile(r"^\[(?P<name>[^\]]+)\]\s*$", re.MULTILINE)


LEFT_ARM_JOINTS = [f"left_arm_joint{i}" for i in range(1, 8)]
RIGHT_ARM_JOINTS = [f"right_arm_joint{i}" for i in range(1, 8)]
TORSO_JOINTS = [f"torso_joint{i}" for i in range(1, 5)]


@dataclass(frozen=True)
class JointLimit:
    joint_type: str
    lower: float | None
    upper: float | None


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def parse_tensor_block(text: str) -> list[list[float]]:
    start = text.find("tensor(")
    if start == -1:
        raise ValueError("Missing tensor(...) block")

    payload = text[start + len("tensor(") :].strip()
    if not payload.endswith(")"):
        raise ValueError("Malformed tensor(...) block")

    payload = payload[:-1].strip()
    parsed = ast.literal_eval(payload)
    if not isinstance(parsed, list):
        raise ValueError("Expected a nested list inside tensor(...)")

    rows: list[list[float]] = []
    for row in parsed:
        if isinstance(row, (int, float)):
            rows.append([float(row)])
            continue
        if not isinstance(row, list):
            raise ValueError("Expected each tensor row to be a list")
        rows.append([float(item) for item in row])
    return rows


def parse_actions_file(path: Path) -> dict[str, list[list[float]]]:
    text = path.read_text()
    matches = list(SECTION_PATTERN.finditer(text))
    if not matches:
        raise ValueError(f"No sections found in {path}")

    parsed: dict[str, list[list[float]]] = {}
    for idx, match in enumerate(matches):
        section = match.group("name").strip()
        section_start = match.end()
        section_end = matches[idx + 1].start() if idx + 1 < len(matches) else len(text)
        section_text = text[section_start:section_end].strip()
        parsed[section] = parse_tensor_block(section_text)
    return parsed


def build_frames(sections: dict[str, list[list[float]]]) -> list[dict[str, list[float]]]:
    frame_counts = {name: len(values) for name, values in sections.items()}
    unique_counts = set(frame_counts.values())
    if len(unique_counts) != 1:
        raise ValueError(f"Section frame counts do not match: {frame_counts}")

    frame_count = unique_counts.pop()
    frames: list[dict[str, list[float]]] = []
    for idx in range(frame_count):
        frame = {name: values[idx] for name, values in sections.items()}
        frames.append(frame)
    return frames


def parse_joint_limits(urdf_path: Path) -> dict[str, JointLimit]:
    root = ET.fromstring(urdf_path.read_text())
    limits: dict[str, JointLimit] = {}

    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        joint_type = joint.attrib["type"]
        limit_el = joint.find("limit")
        lower = float(limit_el.attrib["lower"]) if limit_el is not None and "lower" in limit_el.attrib else None
        upper = float(limit_el.attrib["upper"]) if limit_el is not None and "upper" in limit_el.attrib else None
        limits[name] = JointLimit(joint_type=joint_type, lower=lower, upper=upper)
    return limits


def build_payload(urdf_path: Path, actions_path: Path) -> dict[str, Any]:
    sections = parse_actions_file(actions_path)
    frames = build_frames(sections)
    limits = parse_joint_limits(urdf_path)

    max_gripper_opening = limits["left_gripper_finger_joint1"].upper or 0.05

    return {
        "robot": {
            "name": urdf_path.stem,
            "urdfPath": f"/robot/{urdf_path.name}",
            "urdfDirectory": "/robot/",
            "jointLimits": {
                name: {
                    "type": limit.joint_type,
                    "lower": limit.lower,
                    "upper": limit.upper,
                }
                for name, limit in limits.items()
            },
        },
        "mapping": {
            "left_arm": LEFT_ARM_JOINTS,
            "right_arm": RIGHT_ARM_JOINTS,
            "torso": TORSO_JOINTS,
            "left_gripper": {
                "rawField": "left_gripper",
                "joints": [
                    {"name": "left_gripper_finger_joint1", "direction": 1.0},
                    {"name": "left_gripper_finger_joint2", "direction": -1.0},
                ],
            },
            "right_gripper": {
                "rawField": "right_gripper",
                "joints": [
                    {"name": "right_gripper_finger_joint1", "direction": 1.0},
                    {"name": "right_gripper_finger_joint2", "direction": -1.0},
                ],
            },
        },
        "gripperConfig": {
            "defaultScale": 0.0005,
            "defaultOffset": 0.0,
            "maxOpening": max_gripper_opening,
            "modeHint": "opening = clamp((raw * scale) + offset, 0, maxOpening)",
        },
        "chassisConfig": {
            "enabled": "chassis" in sections,
            "defaultDt": 1.0 / 15.0,
            "defaultLinearScale": 400.0,
            "defaultAngularScale": 1.0,
            "modeHint": "pose += rotate_body_velocity(vx, vy, yaw) * dt * linearScale; yaw += wz * dt * angularScale",
        },
        "sections": sections,
        "frames": frames,
        "frameCount": len(frames),
        "source": {
            "actionsPath": str(actions_path),
            "urdfPath": str(urdf_path),
        },
    }


class VisualizerHandler(BaseHTTPRequestHandler):
    payload: dict[str, Any] = {}
    urdf_root: Path = DEFAULT_URDF_PATH.parent
    static_root: Path = STATIC_DIR

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = unquote(parsed.path)

        if path in {"/", "/index.html"}:
            self._serve_file(self.static_root / "index.html")
            return
        if path == "/app.js":
            self._serve_file(self.static_root / "app.js")
            return
        if path == "/styles.css":
            self._serve_file(self.static_root / "styles.css")
            return
        if path == "/api/data":
            self._send_json(self.payload)
            return
        if path.startswith("/robot/"):
            relative = path.removeprefix("/robot/")
            self._serve_file(self.urdf_root / relative, root=self.urdf_root)
            return

        self.send_error(404, "Not found")

    def _serve_file(self, target: Path, root: Path | None = None) -> None:
        root = root or self.static_root
        resolved_root = root.resolve()
        resolved_target = target.resolve()

        if resolved_root not in resolved_target.parents and resolved_target != resolved_root:
            self.send_error(403, "Forbidden")
            return
        if not resolved_target.exists() or not resolved_target.is_file():
            self.send_error(404, "File not found")
            return

        content_type, _ = mimetypes.guess_type(str(resolved_target))
        self.send_response(200)
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.end_headers()
        self.wfile.write(resolved_target.read_bytes())

    def _send_json(self, payload: dict[str, Any]) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt: str, *args: object) -> None:
        return


def main() -> None:
    parser = argparse.ArgumentParser(description="R1 Pro URDF playback visualizer")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF_PATH)
    parser.add_argument("--actions", type=Path, default=DEFAULT_ACTIONS_PATH)
    args = parser.parse_args()

    payload = build_payload(args.urdf.resolve(), args.actions.resolve())
    VisualizerHandler.payload = payload
    VisualizerHandler.urdf_root = args.urdf.resolve().parent

    server = ThreadingHTTPServer((args.host, args.port), VisualizerHandler)
    print(f"Visualizer serving at http://{args.host}:{args.port}")
    print(f"URDF: {args.urdf.resolve()}")
    print(f"Actions: {args.actions.resolve()}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
