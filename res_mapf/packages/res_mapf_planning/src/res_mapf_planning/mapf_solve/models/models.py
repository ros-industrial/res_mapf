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

from typing import List

from pydantic import BaseModel

"""Models representing MAPF solutions produced by classical MAPF solvers.
"""


class SolverLocation(BaseModel):
    node: str
    x: float | None = None
    y: float | None = None


class Step(BaseModel):
    """
    A step comprises an action to be executed, starting at timestep k, which will change the agent's SolverLocation from step_from to step_to.
    """

    timestep: int
    step_from: SolverLocation
    step_to: SolverLocation
    task_id: str


class SolverPlan(BaseModel):
    """
    An agent's plan from a prior MAPF planner.
    """

    agent_name: str
    steps: List[Step]  # Must not be an empty list.


class Obstacle:
    def __init__(self, location: object) -> None:
        self.location = location

    def __str__(self) -> str:
        return str(self.location)
