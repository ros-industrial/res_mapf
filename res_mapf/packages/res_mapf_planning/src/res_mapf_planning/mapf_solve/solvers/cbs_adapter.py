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

from typing import Dict, List, Sequence, Tuple, TypedDict


from res_mapf_planning.cbs.cbs import CBS, AgentContext, CBSPlan, Environment
from res_mapf_planning.mapf_solve.exceptions import NoSolutionError
from res_mapf_planning.mapf_solve.mapf_solver_base import (
    Location,
    MAPFAgent,
    MAPFSolverBase,
    Obstacle,
)
from res_mapf_planning.mapf_solve.models.models import SolverLocation, SolverPlan, Step


class _CBSOutput(TypedDict):
    schedule: CBSPlan
    cost: int


class CBSAdapter(MAPFSolverBase):
    """Limited adapter to the cbs solver for local testing.
    Missing specification of map dimensions

    start or end should be a Location with Name with a value of a string "x, y"
    """

    def solve(
        self,
        items: Sequence[MAPFAgent],
        input_obstacles: Sequence[Obstacle] = [],
    ) -> Sequence[SolverPlan]:
        task_ids = {}  # Track task IDs

        dimension = [7, 7]
        agents: List[AgentContext] = []

        for item in items:
            agent: AgentContext = {
                "start": self._to_cbs_format(item.start),
                "goal": self._to_cbs_format(item.goal),
                "name": item.agent_id,
            }
            agents.append(agent)

            task_ids[item.agent_id] = item.task_id if item.task_id else ""

        obstacles: List[Tuple[int, int]] = []
        print("Agents:", agents)
        print("Number of obstacles:", len(input_obstacles), flush=True)
        for input_obstacle in input_obstacles:
            print("obstacle:", input_obstacle, flush=True)

            if input_obstacle.location.is_named():
                coords = [
                    int(float(x)) for x in input_obstacle.location.name.split(",")
                ]
                xy_tuple = (coords[0], coords[1])
            else:
                xy_tuple = (
                    int(input_obstacle.location.x),
                    int(input_obstacle.location.y),
                )
            obstacles.append(xy_tuple)
        print("Solving with CBS..", flush=True)
        output = self._solve_cbs(dimension, agents, obstacles)
        plans = _convert_cbs(output, task_ids)
        print("Solved.", flush=True)

        return plans

    def _to_cbs_format(self, location: Location) -> Sequence[int]:
        if location.is_coordinates():
            return [int(location.x), int(location.y)]
        elif location.is_named():
            # assume name is (x, y)
            return [
                int(float(i)) if "." in i else int(i) for i in location.name.split(",")
            ]
        else:
            raise ValueError(f"Invalid Location: {location}")

    def _solve_cbs(
        self,
        dimension: Sequence[int],
        agents: Sequence[AgentContext],
        obstacles: Sequence[Tuple[int, int]],
    ) -> _CBSOutput:
        env = Environment(dimension, agents, obstacles)

        # Searching
        cbs = CBS(env)
        solution = cbs.search()
        if not solution:
            raise NoSolutionError("CBS could not find a valid solution")

        # Write to output file
        output: _CBSOutput = {
            "schedule": solution,
            "cost": env.compute_solution_cost(solution),
        }
        # with open("cbs_output", "w") as output_yaml:
        #     yaml.safe_dump(output, output_yaml)

        return output


def _convert_cbs(
    cbs_output: _CBSOutput, task_ids: Dict[str, str]
) -> Sequence[SolverPlan]:
    """
    Convert output from cbs.py.

    Args:
        cbs_output (dict): Object loaded from cbs.py's output.yaml
    """
    plans = []
    for agent_name, agent_path in cbs_output["schedule"].items():
        steps = []

        # Create steps.

        for idx in range(len(agent_path) - 1):  # Exclude final waypoint from the loop.
            waypoint = agent_path[idx]
            next_waypoint = agent_path[idx + 1]

            timestep = waypoint["t"]
            step_from = SolverLocation(
                node=str(waypoint["x"]) + "," + str(waypoint["y"]),
                x=waypoint["x"],
                y=waypoint["y"],
            )
            step_to = SolverLocation(
                node=str(next_waypoint["x"]) + "," + str(next_waypoint["y"]),
                x=next_waypoint["x"],
                y=next_waypoint["y"],
            )

            steps.append(
                Step(
                    timestep=timestep,
                    step_from=step_from,
                    step_to=step_to,
                    task_id=task_ids[agent_name],
                )
            )
        plans.append(SolverPlan(agent_name=agent_name, steps=steps))

    return plans
