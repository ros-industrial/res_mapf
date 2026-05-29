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

import logging
from typing import Dict, List, Set

from res_plan_server.models.committed_location import CommittedLocation
from res_plan_server.models.task import Task

from res_mapf_planning.mapf_solve.mapf_solver_base import (
    Location,
    MAPFAgent,
    MAPFSolverBase,
    Obstacle,
)
from res_mapf_planning.mapf_solve.models.models import SolverPlan
from res_mapf_planning.planning.multi_agent_context import MultiAgentContext

logger = logging.getLogger("mapf_coordinator")


class PlanningError(Exception):
    """Raised when the coordinator cannot produce a valid plan."""

    pass


class MAPFCoordinator:
    """
    Creates MAPF problem from system state.
    - reads agent states from MultiAgentContext
    - determines start locations from committed locations or agent states
    - determines correct destination
    - converts stationary agents to obstacles
    - calls MAPFSolverABC
    """

    def __init__(self, context: MultiAgentContext, solver: MAPFSolverBase):
        self._context = context
        self._solver = solver

    def solve(
        self,
        new_tasks: List[Task],
        committed_locations: Dict[str, CommittedLocation],
        stationary_agents: Set[str],
        obstacles: List[Obstacle],
    ) -> List[SolverPlan]:
        """Construct and solve a MAPF problem from current state.
        TODO: Handle conflicting goals and obstacles.
        TODO: Return with information on errors

        Args:
            new_tasks (List[Task]): new incoming tasks.
            committed_locations (Dict[str, CommittedLocation]): committed locations received from Executor. empty if this is an initial plan.
            stationary_agents (Set[str]): agents that cannot be moved and to be planned around.
            obstacles (List[Obstacle]): obstacles in the environment.

        Returns:
            List[SolverPlan]: List of solver plans, one per agent.
        """
        task_dict = {d.robot_id: d for d in new_tasks}

        for agent_id in task_dict:
            if not self._context.has_agent(agent_id):
                logger.error(
                    f"Agent {agent_id} has not been initialised - call context's initialise_agent first."
                )
                return []

        context_agents = self._context.get_agents_snapshot()
        mapf_agents: List[MAPFAgent] = []
        for agent_id, entry in context_agents.items():
            if agent_id in stationary_agents:
                continue

            # Determine start location
            if agent_id in committed_locations:
                start = Location(name=committed_locations[agent_id].location)
            elif entry.start_location is not None:
                start = Location(name=entry.start_location)
            else:
                logger.error(
                    f"Agent {agent_id} has no known start location - call initialise_agent first."
                )
                return []

            # Determine goal
            task = task_dict.get(agent_id)

            goal_location = task.goal if task else entry.goal_location
            task_id = task.task_id if task else entry.task_id

            if goal_location is None:
                logger.error(f"Agent {agent_id} has no goal location.")
                return []

            mapf_agents.append(
                MAPFAgent(
                    agent_id=agent_id,
                    start=start,
                    goal=Location(name=goal_location),
                    task_id=task_id,
                )
            )

        if not mapf_agents:
            logger.info("No agents to plan for.")
            return []

        # Convert stationary agents to obstacles. If an agent is stationary, it has reached its goal.
        all_obstacles = list(obstacles)
        for agent_id in stationary_agents:
            entry = context_agents.get(agent_id)
            if entry and entry.goal_location:
                all_obstacles.append(
                    Obstacle(location=Location(name=entry.goal_location))
                )

        # Check for duplicate goals
        goals: Dict[str, str] = {}
        for agent in mapf_agents:
            goal = agent.goal.name
            if goal in goals:
                logger.error(
                    f"Agents {goals[goal]} and {agent.agent_id} have the same goal: {goal}"
                )
                return []
            goals[goal] = agent.agent_id

        self._check_stationary_conflict(mapf_agents, stationary_agents, context_agents)

        solver_plans: List[SolverPlan] = self._solver.solve(mapf_agents, all_obstacles)

        if not solver_plans:
            logger.error("AgenSolver returned no plans.")
            return []

        return solver_plans

    @staticmethod
    def _check_stationary_conflict(
        mapf_agents: List[MAPFAgent],
        stationary_agents: Set[str],
        all_agents: dict,
    ) -> None:
        """Raises PlanningError if a stationary agent blocks a destination."""
        destinations = {a.goal.name for a in mapf_agents if a.goal.is_named()}
        for agent_id in stationary_agents:
            entry = all_agents.get(agent_id)
            if entry and entry.start_location in destinations:
                raise PlanningError(
                    f"Stationary agent {agent_id} is occupying another agent's destination."
                )
