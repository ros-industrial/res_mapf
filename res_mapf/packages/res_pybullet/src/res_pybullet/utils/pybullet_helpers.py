# Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
# Advanced Remanufacturing and Technology Centre
# A*STAR Research Entities (Co. Registration No. 199702110H)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Helper functions for pybullet

from typing import Any


def draw_grid(
    p: Any, xmin: int, xmax: int, ymin: int, ymax: int, interval: float
) -> None:
    """
    Adds text labels for a grid of size m by n to the pybullet simulation

    Args:
        p (_type_): pybullet
        interval (_type_): size of grid along x and y axis
    """
    for i in range(xmin, xmax):
        for j in range(ymin, ymax):
            if i % interval == 0 and j % interval == 0:
                text = "."
                # text = str(i) + ", " + str(j)
                p.addUserDebugText(text, [i, j, 0], lifeTime=0)


def estimate_camera_view(
    p: Any,
    num_agents: int = 0,
    xmin: int = 0,
    xmax: int = 6,
    ymin: int = 0,
    ymax: int = 6,
) -> None:
    # TODO: Estimate suitable camera view based on number of agents or space
    dist = 10
    yaw = 0
    pitch = -55

    x_midpoint = (xmax - xmin) / 2
    y_midpoint = (ymax - ymin) / 2

    target = [x_midpoint, y_midpoint, 0.0]
    p.resetDebugVisualizerCamera(dist, yaw, pitch, target)


def set_camera_view(p: Any, x_target: int, y_target: int) -> None:
    dist = 7
    yaw = 0
    pitch = -55

    target = [x_target, y_target, 0.0]
    p.resetDebugVisualizerCamera(dist, yaw, pitch, target)


def check_escape_key(p: Any) -> bool:
    # Check if Esc was pressed
    key_events = p.getKeyboardEvents()
    return 27 in key_events and key_events[27] & p.KEY_WAS_TRIGGERED
