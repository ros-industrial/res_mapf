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

import abc
from dataclasses import dataclass
from typing import List

from res_mapf_planning.mapf_solve.models.models import SolverPlan


@dataclass(frozen=True)
class Location:
    """
    A position in the environment. Either a named waypoint or x,y coordinates must be provided.
    TODO: Address named vs x,y coordinates.
    """

    name: str | None = None
    x: float | None = None
    y: float | None = None

    def __post_init__(self) -> None:
        if self.name is None and (self.x is None or self.y is None):
            raise ValueError("Location must have either a name or x,y coordinates.")

    def is_named(self) -> bool:
        return self.name is not None

    def is_coordinates(self) -> bool:
        return self.x is not None and self.y is not None

    def __str__(self) -> str:
        if self.name is not None:
            return self.name
        return f"({self.x}, {self.y})"


@dataclass
class MAPFAgent:
    """
    Agent in an MAPF problem.
    """

    agent_id: str
    start: Location
    goal: Location
    task_id: str | None = None


@dataclass(frozen=True)
class Obstacle:
    """A position in the environment that agents cannot occupy."""

    location: Location


class MAPFSolverBase(abc.ABC):
    """
    Interface for a MAPF Solver.
    Given agents with start and goal locations and obstacles, find collision-free paths.

    The `task_id` field of each Step must be provided in the Plan object to enable task tracking.

    Raise:
    SolverConnectionError: if the solver cannot be reached
    NoSolutionFound: if the solver cannot find a solution
    """

    @abc.abstractmethod
    def solve(
        self,
        agents: List[MAPFAgent],
        obstacles: List[Obstacle],
    ) -> List[SolverPlan]:
        pass
