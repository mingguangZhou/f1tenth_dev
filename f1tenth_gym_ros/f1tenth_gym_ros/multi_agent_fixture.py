# MIT License
#
# Copyright (c) 2026 F1TENTH Development Contributors
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Pure validation helpers for deterministic multi-agent simulator fixtures."""

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Mapping, Sequence


TRAFFIC_ARRAY_FIELDS = (
    "traffic_namespaces",
    "traffic_scan_topics",
    "traffic_drive_topics",
    "traffic_start_x",
    "traffic_start_y",
    "traffic_start_theta",
)


@dataclass(frozen=True)
class TrafficAgentSpec:
    """Topics, identity, and initial pose for one simulated traffic car."""

    namespace: str
    scan_topic: str
    drive_topic: str
    start_x: float
    start_y: float
    start_theta: float


def _normalized_topic(value: Any, field: str) -> str:
    name = str(value).strip("/")
    if not name:
        raise ValueError(f"{field} cannot contain blank names")
    return name


def _normalized_namespace(value: Any) -> str:
    namespace = _normalized_topic(value, "traffic_namespaces")
    if "/" in namespace:
        raise ValueError("traffic_namespaces must be single ROS name tokens")
    if namespace == "ego":
        raise ValueError("traffic namespace 'ego' is reserved for diagnostics")
    return namespace


def traffic_agent_specs(
    parameters: Mapping[str, Any]
) -> List[TrafficAgentSpec]:
    """Validate bridge arrays and return their index-aligned traffic specs."""
    num_agents = int(parameters["num_agent"])
    traffic_count = num_agents - 1
    if traffic_count < 1:
        raise ValueError(
            "a multi-agent fixture requires at least one traffic car"
        )

    arrays: Dict[str, Sequence[Any]] = {}
    for field in TRAFFIC_ARRAY_FIELDS:
        values = parameters.get(field)
        valid_array = isinstance(values, (list, tuple))
        if not valid_array or len(values) != traffic_count:
            raise ValueError(
                f"{field} must contain {traffic_count} values for "
                f"num_agent={num_agents}"
            )
        arrays[field] = values

    namespaces = [
        _normalized_namespace(value)
        for value in arrays["traffic_namespaces"]
    ]
    scan_topics = [
        _normalized_topic(value, "traffic_scan_topics")
        for value in arrays["traffic_scan_topics"]
    ]
    drive_topics = [
        _normalized_topic(value, "traffic_drive_topics")
        for value in arrays["traffic_drive_topics"]
    ]
    for field, values in (
        ("traffic_namespaces", namespaces),
        ("traffic_scan_topics", scan_topics),
        ("traffic_drive_topics", drive_topics),
    ):
        if len(set(values)) != traffic_count:
            raise ValueError(f"{field} must be unique")

    odom_leaf = _normalized_topic(
        parameters["opp_odom_topic"], "opp_odom_topic"
    )
    traffic_odom_topics = [
        f"{namespace}/{odom_leaf}" for namespace in namespaces
    ]
    resolved_topics = scan_topics + drive_topics + traffic_odom_topics
    ego_namespace = _normalized_topic(
        parameters.get("ego_namespace", "ego_racecar"), "ego_namespace"
    )
    resolved_topics.extend(
        [
            _normalized_topic(
                parameters.get("ego_scan_topic", "scan"), "ego_scan_topic"
            ),
            _normalized_topic(
                parameters.get("ego_drive_topic", "drive"), "ego_drive_topic"
            ),
            "{}/{}".format(
                ego_namespace,
                _normalized_topic(
                    parameters.get("ego_odom_topic", "odom"),
                    "ego_odom_topic",
                ),
            ),
        ]
    )
    if len(set(resolved_topics)) != len(resolved_topics):
        raise ValueError(
            "resolved scan, drive, and odometry topics must be unique"
        )

    specs = []
    for index in range(traffic_count):
        pose = (
            float(arrays["traffic_start_x"][index]),
            float(arrays["traffic_start_y"][index]),
            float(arrays["traffic_start_theta"][index]),
        )
        if not all(math.isfinite(value) for value in pose):
            raise ValueError("traffic start poses must be finite")
        specs.append(
            TrafficAgentSpec(
                namespace=namespaces[index],
                scan_topic=scan_topics[index],
                drive_topic=drive_topics[index],
                start_x=pose[0],
                start_y=pose[1],
                start_theta=pose[2],
            )
        )
    return specs


def controller_parameters(
    common_parameters: Mapping[str, Any],
    simulator_parameters: Mapping[str, Any],
) -> List[Dict[str, Any]]:
    """Build one independent controller parameter set per traffic car."""
    specs = traffic_agent_specs(simulator_parameters)
    odom_name = _normalized_topic(
        simulator_parameters["opp_odom_topic"], "opp_odom_topic"
    )
    controllers = []
    for index, spec in enumerate(specs):
        parameters = dict(common_parameters)
        parameters.update(
            {
                "agent_name": spec.namespace,
                "odom_topic": f"/{spec.namespace}/{odom_name}",
                "scan_topic": f"/{spec.scan_topic}",
                "drive_topic": f"/{spec.drive_topic}",
                "path_topic": "/moving_agent/reference_path",
                "status_topic": f"/moving_agent/{spec.namespace}/status",
                "publish_reference_path": index == 0,
            }
        )
        controllers.append(parameters)
    return controllers
