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
from itertools import permutations
from typing import Dict, List, Optional

from res_plan_server.models.committed_location import CommittedLocation

from res_mapf_planning.mapf_solve.models.models import SolverPlan
from res_mapf_planning.traffic_dependencies.models.plan import Plan
from res_mapf_planning.traffic_dependencies.models.plan_id import PlanId
from res_mapf_planning.traffic_dependencies.models.traffic_dependency import (
    TrafficDependency,
)
from res_mapf_planning.traffic_dependencies.models.waypoint import Waypoint

logger = logging.getLogger("plan_generator")


class PlanGenerator:
    """
    Minimal implementation for Plan generation.
    Converts solutions from MAPF solver into Plan messages.
    """

    def __init__(self) -> None:
        self.plans_dict: Dict[str, Plan] = {}  # {agent_name: Plan}

    def generate(
        self,
        solver_plans: List[SolverPlan],
        plan_ids: Dict[str, PlanId],
        committed_locations: Optional[Dict[str, CommittedLocation]],
    ) -> List[Plan]:
        self.plans_dict.clear()

        for agent_plan in solver_plans:
            progress_offset = 0.0
            if committed_locations and agent_plan.agent_name in committed_locations:
                progress_offset = committed_locations[agent_plan.agent_name].progress

            if not agent_plan.steps:  # empty plans
                continue

            waypoints = []
            for step in agent_plan.steps:
                wp = Waypoint(
                    name=step.step_from.node,
                    position=[step.step_from.x, step.step_from.y],
                    progress=float(step.timestep) + progress_offset,
                    departure_blockers=[],
                    departure_action="",
                )
                waypoints.append(wp)
            last_step = agent_plan.steps[-1]
            final_wp = Waypoint(
                name=last_step.step_to.node,
                position=[last_step.step_to.x, last_step.step_to.y],
                progress=float(last_step.timestep + 1)
                + progress_offset,  # account for final waypoint
                departure_blockers=[],
                departure_action="",
            )
            waypoints.append(final_wp)

            plan = Plan(
                waypoints=waypoints,
                start_time=None,
                plan_id=plan_ids[agent_plan.agent_name],
                workflow="",
            )
            self.plans_dict[agent_plan.agent_name] = plan

        self._add_traffic_dependencies(solver_plans, plan_ids)

        return list(self.plans_dict.values())

    def _add_traffic_dependencies(
        self, solver_plans: List[SolverPlan], plan_ids: Dict[str, PlanId]
    ) -> None:
        """
        Add inter-robot dependencies.
        If agent i leaves location L at a timestep earlier or equal to when agent j arrives,
        add TrafficDependency to j to ensure that i has vacated L.
        """
        for solver_plan_i, solver_plan_j in permutations(solver_plans, 2):
            steps_i = solver_plan_i.steps
            steps_j = solver_plan_j.steps

            for k_i, step_i in enumerate(steps_i):
                for k_j, step_j in enumerate(steps_j):
                    if (
                        step_i.step_from == step_j.step_to
                        and step_i.timestep <= step_j.timestep
                    ):
                        # Plan.waypoints[step_j] corresponds to SolverPlan.steps[step_j].step_from
                        # Block agent j from departing this waypoint until agent i arrives at step_i's step_to
                        blocker = TrafficDependency(
                            name=solver_plan_i.agent_name,
                            plan_id=plan_ids[solver_plan_i.agent_name],
                            required_progress=self.plans_dict[solver_plan_i.agent_name]
                            .waypoints[k_i + 1]
                            .progress,
                            # require agent i to finish destination Waypoint
                        )
                        plan_j = self.plans_dict[solver_plan_j.agent_name]
                        plan_j.waypoints[k_j].departure_blockers.append(blocker)
                        break
