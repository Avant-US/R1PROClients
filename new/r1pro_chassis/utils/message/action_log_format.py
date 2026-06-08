def _format_value(value):
    if value is None:
        return "None"
    return f"{float(value):.4f}"


def _format_values(values):
    return "[" + ", ".join(_format_value(value) for value in values) + "]"


def _joint_positions(msg, size):
    if msg is None:
        return [None] * size

    values = list(getattr(msg, "position", []))
    return (values + [None] * size)[:size]


def _chassis_values(msg):
    if msg is None:
        return [None, None, None]

    twist = getattr(msg, "twist", None)
    linear = getattr(twist, "linear", None)
    angular = getattr(twist, "angular", None)
    return [
        getattr(linear, "x", None),
        getattr(linear, "y", None),
        getattr(angular, "z", None),
    ]


def format_joint_action_23d(action) -> str:
    values = []
    values.extend(_joint_positions(action.left_arm, 7))
    values.extend(_joint_positions(action.right_arm, 7))
    values.extend(_joint_positions(action.left_gripper, 1))
    values.extend(_joint_positions(action.right_gripper, 1))
    values.extend(_joint_positions(action.torso, 4))
    values.extend(_chassis_values(action.chassis))

    return (
        "action_23d:\n"
        f"  left_arm      [0:7]   = {_format_values(values[0:7])}\n"
        f"  right_arm     [7:14]  = {_format_values(values[7:14])}\n"
        f"  left_gripper  [14:15] = {_format_values(values[14:15])}\n"
        f"  right_gripper [15:16] = {_format_values(values[15:16])}\n"
        f"  torso         [16:20] = {_format_values(values[16:20])}\n"
        f"  chassis       [20:23] = {_format_values(values[20:23])}"
    )
