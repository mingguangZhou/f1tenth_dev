import numpy as np


def speed_index_to_mps(speed_index: float, min_speed: float = 0.5, max_speed: float = 4.0) -> float:
    """
    Convert normalized speed_index [0, 1] to physical speed [m/s].
    """
    speed_index = float(np.clip(speed_index, 0.0, 1.0))
    return float(min_speed + speed_index * (max_speed - min_speed))


def speed_mps_to_index(speed: float, min_speed: float = 0.5, max_speed: float = 4.0) -> float:
    """
    Convert physical speed [m/s] to normalized speed_index [0, 1].
    """
    if max_speed <= min_speed:
        raise ValueError("max_speed must be larger than min_speed")
    return float(np.clip((speed - min_speed) / (max_speed - min_speed), 0.0, 1.0))


def clamp_speed(speed: float, min_speed: float = 0.5, max_speed: float = 4.0) -> float:
    """
    Clamp physical speed to the configured range.
    """
    return float(np.clip(speed, min_speed, max_speed))


def curvature_based_speed(
    upcoming_curvature_abs: float,
    min_speed: float = 0.5,
    max_speed: float = 4.0,
    curvature_gain: float = 2.0,
) -> float:
    """
    Physical-speed version of the simple curvature rule.

    Low curvature -> high speed.
    High curvature -> low speed.
    """
    raw_speed = max_speed / (1.0 + curvature_gain * upcoming_curvature_abs)
    return clamp_speed(raw_speed, min_speed, max_speed)


def curvature_based_speed_index(
    upcoming_curvature_abs: float,
    min_speed: float = 0.5,
    max_speed: float = 4.0,
    curvature_gain: float = 2.0,
) -> float:
    """
    Normalized speed-index version of the simple curvature baseline.

    This is useful for comparing the PPO policy with a rule-based policy through
    the same F110SpeedEnv action interface.
    """
    speed = curvature_based_speed(
        upcoming_curvature_abs=upcoming_curvature_abs,
        min_speed=min_speed,
        max_speed=max_speed,
        curvature_gain=curvature_gain,
    )
    return speed_mps_to_index(speed, min_speed=min_speed, max_speed=max_speed)
