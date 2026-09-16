"""Validation truth must match the normal bridge and never use raw odometry."""

import copy
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticStatus

from f1tenth_gym_ros.gym_bridge import GymBridge as NormalBridge
from f1tenth_gym_ros.gym_bridge_slam import GymBridge as SlamBridge


@pytest.mark.parametrize('has_opp', [False, True])
@pytest.mark.parametrize('with_collisions', [False, True])
def test_slam_truth_contract_matches_normal_bridge(has_opp, with_collisions):
    obs = {
        'poses_x': [10.0, 13.0], 'poses_y': [-2.0, 2.0],
        'poses_theta': [0.7, -0.2], 'linear_vels_x': [-3.0, 0.0],
        'linear_vels_y': [4.0, 0.0], 'ang_vels_z': [0.25, -0.1],
    }
    if with_collisions:
        obs['collisions'] = [True, False]
    before = copy.deepcopy(obs)
    bridge = SimpleNamespace(
        obs=obs, has_opp=has_opp, ego_namespace='ego_racecar',
        opp_namespace='opp_racecar', agent_status_pub=Mock(),
        odom_x=999.0, odom_y=888.0, odom_yaw=2.0,
        ego_pose=[777.0, 666.0, 1.0], ego_speed=[55.0, 44.0, 3.0])
    stamp = Time(sec=42, nanosec=123)
    SlamBridge._publish_agent_status(bridge, stamp)
    message = bridge.agent_status_pub.publish.call_args[0][0]
    assert message.header.stamp == stamp
    assert message.header.frame_id == ''  # Existing diagnostic contract.
    assert len(message.status) == (2 if has_opp else 1)
    ego = message.status[0]
    assert ego.name == 'simulator/ego'
    assert ego.hardware_id == 'f1tenth_gym'
    assert ego.level == (DiagnosticStatus.ERROR if with_collisions else DiagnosticStatus.OK)
    assert ego.message == ('COLLISION' if with_collisions else 'DRIVING')
    values = {item.key: item.value for item in ego.values}
    assert values == {
        'collision': str(with_collisions), 'x_m': '10.0', 'y_m': '-2.0',
        'yaw_rad': '0.7', 'speed_mps': '5.0', 'body_speed_mps': '-3.0',
        'yaw_rate_radps': '0.25',
        'agent_separation_m': '5.0' if has_opp else 'inf',
    }
    for key in ('x_m', 'y_m', 'yaw_rad', 'speed_mps', 'body_speed_mps', 'yaw_rate_radps'):
        assert math.isfinite(float(values[key]))
    if has_opp:
        assert message.status[1].name == 'simulator/slow_agent'
        assert message.status[1].level == DiagnosticStatus.OK
        assert dict((v.key, v.value) for v in message.status[1].values)['agent_separation_m'] == '5.0'
    normal = SimpleNamespace(
        agent_namespaces=['ego_racecar', 'opp_racecar'][:len(message.status)],
        poses=[[10.0, -2.0, 0.7], [13.0, 2.0, -0.2]],
        speeds=[[-3.0, 4.0, 0.25], [0.0, 0.0, -0.1]],
        collisions=[with_collisions, False], agent_status_pub=Mock())
    normal.poses = normal.poses[:len(message.status)]
    normal.speeds = normal.speeds[:len(message.status)]
    NormalBridge._publish_agent_status(normal, stamp)
    assert normal.agent_status_pub.publish.call_args[0][0] == message
    assert obs == before
    assert (bridge.odom_x, bridge.odom_y, bridge.odom_yaw) == (999.0, 888.0, 2.0)


def test_normal_multi_agent_names_and_nearest_separation():
    from f1tenth_gym_ros.agent_status import make_agent_status

    message = make_agent_status(
        Time(), ['ego_racecar', 'traffic_a', 'traffic_b'],
        [[0.0, 0.0, 0.0], [3.0, 4.0, 0.0], [0.0, 2.0, 0.0]],
        [[0.0, 0.0, 0.0]] * 3, [False] * 3)
    assert [status.name for status in message.status] == [
        'simulator/ego', 'simulator/traffic_a', 'simulator/traffic_b']
    separations = [
        float(next(v.value for v in status.values if v.key == 'agent_separation_m'))
        for status in message.status]
    assert separations == pytest.approx([2.0, math.sqrt(13.0), 2.0])
