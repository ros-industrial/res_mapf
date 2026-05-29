# Copyright (C) 2025-2026 ROS-Industrial Consortium Asia Pacific
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

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, List

from res_mapf_planning.mapf_solve.mapf_solver_base import Location


@dataclass
class WaypointWithCallback:
    """A waypoint to be executed by the robot. on_reached must be called when the robot arrives."""

    location: Location  # TODO: perhaps a common package
    on_reached: Callable[[], None]


class BaseRobotController(ABC):
    """Subclasses must implement enqueue() and shutdown()."""

    @abstractmethod
    def enqueue(
        self, robot_id: str, waypoints_with_callbacks: List[WaypointWithCallback]
    ) -> None:
        """
        Implement this method to handle WaypointWithCallback-s.
        Robots are expected to move to waypoints in FIFO order, calling on_reached for each.
        """
        ...

    @abstractmethod
    def shutdown(self, interrupted: bool = False) -> None:
        """This method will be called when execution is interrupted or completed successfully."""
        ...
