from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from utils.message.action_log_format import format_joint_action_23d


def joint_state(*positions):
    return SimpleNamespace(position=list(positions))


def twist_stamped(vx, vy, wz):
    return SimpleNamespace(
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=vx, y=vy),
            angular=SimpleNamespace(z=wz),
        )
    )


class ActionLogFormatTest(unittest.TestCase):
    def test_formats_joint_action_as_23d_blocks(self):
        action = SimpleNamespace(
            left_arm=joint_state(0, 1, 2, 3, 4, 5, 6),
            right_arm=joint_state(7, 8, 9, 10, 11, 12, 13),
            left_gripper=joint_state(14),
            right_gripper=joint_state(15),
            torso=joint_state(16, 17, 18, 19),
            chassis=twist_stamped(20, 21, 22),
        )

        formatted = format_joint_action_23d(action)

        self.assertIn("action_23d:", formatted)
        self.assertIn("left_arm      [0:7]   = [0.0000, 1.0000", formatted)
        self.assertIn("right_arm     [7:14]  = [7.0000, 8.0000", formatted)
        self.assertIn("left_gripper  [14:15] = [14.0000]", formatted)
        self.assertIn("right_gripper [15:16] = [15.0000]", formatted)
        self.assertIn("torso         [16:20] = [16.0000, 17.0000, 18.0000, 19.0000]", formatted)
        self.assertIn("chassis       [20:23] = [20.0000, 21.0000, 22.0000]", formatted)

    def test_formats_missing_fields_as_none(self):
        action = SimpleNamespace(
            left_arm=None,
            right_arm=None,
            left_gripper=None,
            right_gripper=None,
            torso=None,
            chassis=None,
        )

        formatted = format_joint_action_23d(action)

        self.assertIn("left_arm      [0:7]   = [None, None, None, None, None, None, None]", formatted)
        self.assertIn("chassis       [20:23] = [None, None, None]", formatted)


if __name__ == "__main__":
    unittest.main()
