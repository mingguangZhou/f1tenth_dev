#!/usr/bin/env python3
"""CSV loading helpers for offline-generated centerline data."""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Tuple


REQUIRED_COLUMNS = ('index', 'x', 'y')


@dataclass
class CenterlinePoint:
    """One row of centerline data.

    Known fields are promoted to attributes. Any extra CSV columns are stored in
    ``extras`` so the ROS layer remains forward-compatible with future offline
    exports such as curvature, heading, width, speed hint, and so on.
    """

    index: int
    x: float
    y: float
    extras: Dict[str, float | str]


def _parse_numeric_or_string(value: str):
    value = value.strip()
    if value == '':
        return value
    try:
        parsed = float(value)
        if math.isfinite(parsed):
            return parsed
        raise ValueError(f'Non-finite numeric value: {value}')
    except ValueError:
        return value


def load_centerline_csv(csv_path: str) -> Tuple[List[CenterlinePoint], List[str]]:
    """Load centerline rows from CSV.

    Returns:
        points: Ordered centerline points.
        extra_columns: Column names beyond index/x/y for future use.
    """
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f'Centerline CSV not found: {csv_path}')

    with open(csv_path, 'r', newline='') as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError('CSV has no header row.')

        fieldnames = [name.strip() for name in reader.fieldnames]
        missing = [col for col in REQUIRED_COLUMNS if col not in fieldnames]
        if missing:
            raise ValueError(
                f'CSV is missing required columns {missing}. Found columns: {fieldnames}'
            )

        extra_columns = [col for col in fieldnames if col not in REQUIRED_COLUMNS]
        points: List[CenterlinePoint] = []

        for row_number, row in enumerate(reader, start=2):
            try:
                index = int(row['index'])
                x = float(row['x'])
                y = float(row['y'])
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f'Failed to parse required values at CSV row {row_number}: {row}'
                ) from exc

            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError(f'Non-finite x/y at CSV row {row_number}: {row}')

            extras = {
                col: _parse_numeric_or_string(row[col])
                for col in extra_columns
            }
            points.append(CenterlinePoint(index=index, x=x, y=y, extras=extras))

    if len(points) < 2:
        raise ValueError(
            f'Centerline CSV must contain at least 2 data points. Got {len(points)}.'
        )

    return points, extra_columns


def is_closed_loop(points: List[CenterlinePoint], tolerance_m: float) -> bool:
    """Return True when the first and last points are within tolerance."""
    if len(points) < 2:
        return False

    dx = points[0].x - points[-1].x
    dy = points[0].y - points[-1].y
    return math.hypot(dx, dy) <= tolerance_m


def close_loop(points: List[CenterlinePoint]) -> List[CenterlinePoint]:
    """Append a copy of the first point to the end, preserving extras."""
    if not points:
        return points

    first = points[0]
    closed_points = list(points)
    closed_points.append(
        CenterlinePoint(
            index=len(points),
            x=first.x,
            y=first.y,
            extras=dict(first.extras),
        )
    )
    return closed_points
