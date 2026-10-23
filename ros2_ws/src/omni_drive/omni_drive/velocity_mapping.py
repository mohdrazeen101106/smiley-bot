"""Hardware-independent joystick axis mapping for normalized Blitz Velocity fields."""
from math import isfinite


def clamp(value: float, low: float = -1.0, high: float = 1.0) -> float:
    value = float(value)
    if not isfinite(value):
        return 0.0
    return max(low, min(high, value))


def shape_axis(value: float, deadzone: float) -> float:
    """Apply symmetric deadzone and stretch remaining input to [-1, 1]."""
    if not 0.0 <= float(deadzone) < 1.0: raise ValueError("deadzone must be in [0, 1)")
    value = clamp(value)
    magnitude = abs(value)
    if magnitude <= deadzone: return 0.0
    shaped = (magnitude - deadzone) / (1.0 - deadzone)
    return shaped if value > 0 else -shaped


def map_velocity(
    forward_axis: float,
    lateral_axis: float,
    yaw_input: float,
    *,
    deadzone: float = 0.05,
    forward_sign: float = -1.0,
    lateral_sign: float = -1.0,
    yaw_sign: float = 1.0,
) -> tuple[float, float, float]:
    """Return normalized (vx-forward, vy-left, vw-counterclockwise) values."""
    if not 0.0 <= float(deadzone) < 1.0:
        raise ValueError("deadzone must be in [0, 1)")
    for name, sign in (("forward_sign", forward_sign), ("lateral_sign", lateral_sign), ("yaw_sign", yaw_sign)):
        if not isfinite(float(sign)) or float(sign) not in (-1.0, 1.0):
            raise ValueError(f"{name} must be +1.0 or -1.0")
    vx = shape_axis(forward_axis, deadzone) * float(forward_sign)
    vy = shape_axis(lateral_axis, deadzone) * float(lateral_sign)
    vw = shape_axis(yaw_input, deadzone) * float(yaw_sign)
    return clamp(vx), clamp(vy), clamp(vw)
