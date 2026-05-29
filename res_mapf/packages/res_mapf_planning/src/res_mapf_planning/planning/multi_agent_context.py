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

import copy
import logging
import threading
from enum import Enum, auto
from typing import Dict, List, Optional

from res_plan_server.models.committed_location import CommittedLocation
from res_plan_server.models.task import Task

from res_mapf_planning.traffic_dependencies.models.plan_id import PlanId

logger = logging.getLogger("multi_agent_context")


class AgentStatus(Enum):
    IDLE = auto()  # no active task, available for new requests
    EXECUTING = auto()  # executing a task


class AgentEntry:
    def __init__(self, agent_id: str):
        self.agent_id = agent_id
        self.status = AgentStatus.IDLE
        self.task_id: Optional[str] = None
        self.plan_id: Optional[PlanId] = None
        self.start_location: Optional[str] = None
        self.goal_location: Optional[str] = None

    def __repr__(self) -> str:
        return (
            f"AgentEntry({self.agent_id}, {self.status.name}, "
            f"start={self.start_location}, goal={self.goal_location})"
        )


class MultiAgentContext:
    """
    Maintain state for all agents.
    """

    def __init__(self) -> None:
        self._agents: Dict[str, AgentEntry] = {}
        self._lock = threading.Lock()

    def initialise_agent(self, agent_id: str, start_location: str) -> None:
        """
        Initialise an agent at a known start location.
        Repeated calls will override the start location.
        """
        with self._lock:
            if agent_id not in self._agents:
                self._agents[agent_id] = AgentEntry(agent_id)
            else:
                logger.info("Re-initialising agent %s..", agent_id)

            self._agents[agent_id].start_location = start_location
            self._agents[
                agent_id
            ].goal_location = start_location  # Or leave empty and handle as obstacle?
            logger.info("Initialised agent %s at %s", agent_id, start_location)

    def has_agent(self, agent_id: str) -> bool:
        with self._lock:
            return agent_id in self._agents

    def get_agent_snapshot(self, agent_id: str) -> Optional[AgentEntry]:
        with self._lock:
            return copy.deepcopy(self._agents.get(agent_id))

    def get_agents_snapshot(self) -> Dict[str, AgentEntry]:
        with self._lock:
            return copy.deepcopy(self._agents)

    def has_executing_agents(self) -> bool:
        with self._lock:
            return any(
                entry.status == AgentStatus.EXECUTING for entry in self._agents.values()
            )

    def on_solve_success(
        self,
        committed_locations: Dict[str, CommittedLocation],
        new_tasks: List[Task],
        plan_ids: Dict[str, PlanId],
    ) -> None:
        """
        Called after a successful solve to apply updates.
        Updates start locations from committed locations and goals from new destinations.
        """
        new_task_map = {d.robot_id: d for d in new_tasks}

        # Apply new start locations
        for agent_id, location in committed_locations.items():
            entry = self._agents.get(agent_id)
            if entry is not None:
                entry.start_location = location.location

        # Apply new destinations.
        for agent_id, task in new_task_map.items():
            entry = self._agents.get(agent_id)
            if entry is not None:
                entry.task_id = task.task_id
                entry.plan_id = plan_ids[task.robot_id]
                entry.goal_location = task.goal
                entry.status = AgentStatus.EXECUTING

    def on_completed(self, agent_id: str, plan_id: PlanId) -> None:
        """
        Agent has reached its goal.
        """
        entry = self._agents.get(agent_id)
        if entry is None:
            logger.warning(
                "Completion for task %s ignored, agent %s is not in context.",
                plan_id,
                agent_id,
            )
            return

        if entry.plan_id != plan_id:
            logger.warning(
                "Completion for task %s ignored, agent %s is on task %s",
                plan_id,
                agent_id,
                entry.plan_id,
            )
            return

        entry.start_location = entry.goal_location
        entry.plan_id = None
        entry.goal_location = None
        entry.status = AgentStatus.IDLE
        logger.info("Agent %s completed plan %s", agent_id, plan_id)

    def on_failed(self, agent_id: str, plan_id: str) -> None:
        """
        Clear stored start location
        """
        entry = self._agents.get(agent_id)
        if entry is None:
            return

        if entry.plan_id != plan_id:
            logger.warning(
                "Failure for task %s ignored, agent %s is on task %s",
                plan_id,
                agent_id,
                entry.plan_id,
            )
            return

        entry.start_location = None
        entry.plan_id = None
        entry.goal_location = None
        entry.status = AgentStatus.IDLE

        logger.warning("Agent %s failed task %s.", agent_id, plan_id)
