import math
from typing import List, Optional, Tuple

from rl_training.centerline_utils import (
    CenterlinePoint,
    find_nearest_centerline_index,
    get_loop_index,
)


def distance_2d(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> float:
    """
    Compute 2D Euclidean distance.
    """
    dx = x1 - x2
    dy = y1 - y2
    return math.sqrt(dx * dx + dy * dy)


def compute_speed_dependent_lookahead(
    speed: float,
    min_lookahead: float = 0.6,
    max_lookahead: float = 1.6,
    speed_gain: float = 0.25,
) -> float:
    """
    Compute lookahead distance based on current vehicle speed.

    Logic:
        Low speed:
            shorter lookahead, so the controller tracks corners more tightly.

        High speed:
            longer lookahead, so steering is smoother and less nervous.

    Formula:
        lookahead = min_lookahead + speed_gain * speed

    Then the result is clamped to:
        [min_lookahead, max_lookahead]
    """
    lookahead = min_lookahead + speed_gain * max(0.0, speed)
    return max(min_lookahead, min(max_lookahead, lookahead))


def find_lookahead_index(
    car_x: float,
    car_y: float,
    centerline: List[CenterlinePoint],
    lookahead_distance: float,
) -> int:
    """
    Find a centerline point approximately lookahead_distance ahead of the car.

    Method:
        1. Find the nearest centerline point to the car.
        2. Walk forward along the closed-loop centerline.
        3. Return the first point whose Euclidean distance from the car
           is greater than or equal to lookahead_distance.

    This is intentionally simple and robust for the first RL pipeline.
    """
    nearest_idx = find_nearest_centerline_index(
        x=car_x,
        y=car_y,
        centerline=centerline,
    )

    n = len(centerline)

    for offset in range(n):
        idx = get_loop_index(nearest_idx + offset, n)
        p = centerline[idx]

        distance = distance_2d(
            car_x,
            car_y,
            p.x,
            p.y,
        )

        if distance >= lookahead_distance:
            return idx

    # Fallback: should rarely happen on a valid closed-loop centerline.
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
        x-axis: forward
        y-axis: left

    Pure pursuit is easiest to compute when the lookahead target point
    is expressed relative to the vehicle.
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
    current_speed: Optional[float] = None,
    use_speed_dependent_lookahead: bool = False,
    min_lookahead: float = 0.6,
    max_lookahead: float = 1.6,
    lookahead_speed_gain: float = 0.25,
) -> Tuple[float, int]:
    """
    Compute steering angle using pure pursuit.

    There are two modes:

    1. Fixed lookahead:
        use_speed_dependent_lookahead = False

        The controller uses lookahead_distance directly.

    2. Speed-dependent lookahead:
        use_speed_dependent_lookahead = True

        The controller computes:
            active_lookahead = min_lookahead + lookahead_speed_gain * current_speed

        Then clamps it to:
            [min_lookahead, max_lookahead]

    Returns:
        steering:
            steering angle in radians

        lookahead_idx:
            selected lookahead point index on the centerline
    """
    if use_speed_dependent_lookahead:
        if current_speed is None:
            raise ValueError(
                "current_speed must be provided when "
                "use_speed_dependent_lookahead=True"
            )

        active_lookahead_distance = compute_speed_dependent_lookahead(
            speed=current_speed,
            min_lookahead=min_lookahead,
            max_lookahead=max_lookahead,
            speed_gain=lookahead_speed_gain,
        )
    else:
        active_lookahead_distance = lookahead_distance

    lookahead_idx = find_lookahead_index(
        car_x=car_x,
        car_y=car_y,
        centerline=centerline,
        lookahead_distance=active_lookahead_distance,
    )

    target = centerline[lookahead_idx]

    local_x, local_y = transform_point_to_vehicle_frame(
        point_x=target.x,
        point_y=target.y,
        car_x=car_x,
        car_y=car_y,
        car_yaw=car_yaw,
    )

    # If the target is behind the vehicle, the pure pursuit formula can become
    # unstable. This guard keeps the controller safe during resets or bad states.
    if local_x <= 1e-6:
        return 0.0, lookahead_idx

    # Pure pursuit geometry:
    #
    #   curvature = 2 * lateral_offset / lookahead_distance^2
    #   steering  = atan(wheelbase * curvature)
    #
    # Here we compute lookahead_distance^2 from the actual local target point.
    ld_sq = local_x * local_x + local_y * local_y
    curvature = 2.0 * local_y / ld_sq

    steering = math.atan(wheelbase * curvature)

    # Clamp to simulator / vehicle steering limit.
    steering = max(-max_steer, min(max_steer, steering))

    return steering, lookahead_idx