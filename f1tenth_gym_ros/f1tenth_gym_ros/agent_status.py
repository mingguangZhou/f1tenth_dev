# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def make_agent_status(stamp, namespaces, poses, speeds, collisions):
    """Encode simulator truth for validation, never estimated/raw odometry.

    Preserve the normal bridge contract, including infinite separation when
    there is no other agent. Pose is in simulator map coordinates; speeds are
    the simulator observation's linear x/y and angular z values.
    """
    message = DiagnosticArray()
    message.header.stamp = stamp
    statuses = []
    for index, (namespace, pose, speed, collision) in enumerate(zip(
            namespaces, poses,
            speeds, collisions)):
        name = 'ego' if index == 0 else namespace
        if len(namespaces) == 2 and index == 1:
            name = 'slow_agent'
        separation = min(
            (math.hypot(
                pose[0] - other_pose[0],
                pose[1] - other_pose[1])
             for other_index, other_pose in enumerate(poses)
             if other_index != index),
            default=math.inf)
        status = DiagnosticStatus()
        status.name = f'simulator/{name}'
        status.hardware_id = 'f1tenth_gym'
        status.level = (
            DiagnosticStatus.ERROR if collision else DiagnosticStatus.OK)
        status.message = 'COLLISION' if collision else 'DRIVING'
        values = {
            'collision': collision,
            'x_m': pose[0],
            'y_m': pose[1],
            'yaw_rad': pose[2],
            'speed_mps': math.hypot(speed[0], speed[1]),
            'body_speed_mps': speed[0],
            'yaw_rate_radps': speed[2],
            'agent_separation_m': separation,
        }
        status.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in values.items()]
        statuses.append(status)
    message.status = statuses
    return message
