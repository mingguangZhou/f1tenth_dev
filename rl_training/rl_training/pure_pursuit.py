import math
from typing import List, Tuple

from rl_training.centerline_utils import (
    CenterlinePoint,
    find_nearest_centerline_index,
    get_loop_index,
    wrap_angle,
)


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """
    Euclidean distance in 2D.
    """
    dx = x1 - x2
    dy = y1 - y2
    return math.sqrt(dx * dx + dy * dy)


def find_lookahead_index(
    car_x: float,
    car_y: float,
    centerline: List[CenterlinePoint],
    lookahead_distance: float,
) -> int:
    """
    Find a centerline point approximately lookahead_distance ahead of the car.

    Method:
    1. Find nearest centerline point.
    2. Move forward along the closed-loop centerline.
    3. Return the first point whose distance from the car is >= lookahead_distance.

    This is simple and robust enough for the first RL training pipeline.
    """
    nearest_idx = find_nearest_centerline_index(car_x, car_y, centerline)
    n = len(centerline)

    for offset in range(n):
        idx = get_loop_index(nearest_idx + offset, n)
        p = centerline[idx]

        if distance_2d(car_x, car_y, p.x, p.y) >= lookahead_distance:
            return idx

    # Fallback should almost never happen on a normal closed track.
    return nearest_idx


def transform_point_to_vehicle_frame(
    point_x: float,
    point_y: float,
    car_x: float,
    car_y: float,
    car_yaw: float,
) -> Tuple[float, float]:
    """
    Transform a world-frame point into the vehicle frame.

    Vehicle frame convention:
        x forward
        y left

    This is needed because pure pursuit steering is easiest to compute
    using the lookahead point relative to the vehicle.
    """
    dx = point_x - car_x
    dy = point_y - car_y

    cos_yaw = math.cos(car_yaw)
    sin_yaw = math.sin(car_yaw)

    # Rotate world delta by -car_yaw.
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy

    return local_x, local_y


def compute_pure_pursuit_steering(
    car_x: float,
    car_y: float,
    car_yaw: float,
    centerline: List[CenterlinePoint],
    lookahead_distance: float = 1.0,
    wheelbase: float = 0.33,
    max_steer: float = 0.4189,
) -> Tuple[float, int]:
    """
    Compute steering angle using pure pursuit.

    Args:
        car_x, car_y, car_yaw:
            Current vehicle pose in map/world frame.

        centerline:
            Loaded centerline waypoints.

        lookahead_distance:
            Target lookahead distance in meters.

        wheelbase:
            Vehicle wheelbase in meters.
            For F1TENTH cars this is often around 0.32-0.34 m.

        max_steer:
            Steering clamp in radians.
            0.4189 rad is about 24 degrees.

    Returns:
        steering_angle:
            Steering command in radians.

        lookahead_idx:
            Index of selected lookahead point.
    """
    lookahead_idx = find_lookahead_index(
        car_x=car_x,
        car_y=car_y,
        centerline=centerline,
        lookahead_distance=lookahead_distance,
    )

    target = centerline[lookahead_idx]

    local_x, local_y = transform_point_to_vehicle_frame(
        point_x=target.x,
        point_y=target.y,
        car_x=car_x,
        car_y=car_y,
        car_yaw=car_yaw,
    )

    # If the selected target is behind the vehicle, steering becomes unstable.
    # In normal operation this should not happen often, but this guard is useful
    # during early testing and resets.
    if local_x <= 1e-6:
        return 0.0, lookahead_idx

    # Pure pursuit formula:
    # curvature = 2 * y / Ld^2
    # steering = atan(wheelbase * curvature)
    #
    # Here Ld is the distance from vehicle to target point.
    ld_sq = local_x * local_x + local_y * local_y
    curvature = 2.0 * local_y / ld_sq

    steering = math.atan(wheelbase * curvature)

    # Clamp steering to simulator / vehicle limit.
    steering = max(-max_steer, min(max_steer, steering))

    return steering, lookahead_idx
