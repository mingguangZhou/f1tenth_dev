import numpy as np


def clamp_speed(speed: float, min_speed: float = 0.5, max_speed: float = 4.0) -> float:
    """
    Clamp speed to the same range used by the future RL agent.
    """
    return float(np.clip(speed, min_speed, max_speed))


def curvature_based_speed(
    upcoming_curvature_abs: float,
    min_speed: float = 0.5,
    max_speed: float = 4.0,
    curvature_gain: float = 2.0,
) -> float:
    """
    Low curvature -> high speed.
    High curvature -> low speed.
    """
    raw_speed = max_speed / (1.0 + curvature_gain * upcoming_curvature_abs)
    return clamp_speed(raw_speed, min_speed, max_speed)
