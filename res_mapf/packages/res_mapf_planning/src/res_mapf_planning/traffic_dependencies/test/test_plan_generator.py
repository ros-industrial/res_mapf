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

import uuid

from res_mapf_planning.mapf_solve.models.models import SolverLocation, SolverPlan, Step
from res_mapf_planning.traffic_dependencies.models.plan import Plan
from res_mapf_planning.traffic_dependencies.models.plan_id import PlanId
from res_mapf_planning.traffic_dependencies.models.traffic_dependency import (
    TrafficDependency,
)
from res_mapf_planning.traffic_dependencies.models.waypoint import Waypoint
from res_mapf_planning.traffic_dependencies.plan_generator import PlanGenerator


def test_generate_with_conflict() -> None:
    # ------------------
    # Input
    # ------------------

    agent_a = SolverPlan(
        agent_name="A",
        steps=[
            Step(
                timestep=0,
                step_from=SolverLocation(node="X", x=0.0, y=0.0),
                step_to=SolverLocation(node="Y", x=1.0, y=0.0),
                task_id="A_task",
            )
        ],
    )

    agent_b = SolverPlan(
        agent_name="B",
        steps=[
            Step(
                timestep=0,
                step_from=SolverLocation(node="Z", x=0.0, y=1.0),
                step_to=SolverLocation(node="X", x=0.0, y=0.0),
                task_id="B_task",
            )
        ],
    )

    solver_plans = [agent_a, agent_b]

    plan_ids = {
        "A": PlanId(uuid.uuid4(), 0),
        "B": PlanId(uuid.uuid4(), 0),
    }

    # Simulate previously committed progress
    committed_locations = {
        "A": Waypoint(
            name="committed_A",
            position=[0.0, 0.0],
            progress=5.0,
            departure_blockers=[],
            departure_action="",
        ),
        "B": Waypoint(
            name="committed_B",
            position=[0.0, 1.0],
            progress=10.0,
            departure_blockers=[],
            departure_action="",
        ),
    }

    generator = PlanGenerator()
    generated = generator.generate(solver_plans, plan_ids, committed_locations)

    # ------------------
    # Expected Plans
    # ------------------

    # Agent A expected
    expected_a = Plan(
        plan_id=plan_ids["A"],
        start_time=None,
        workflow="",
        waypoints=[
            Waypoint(
                name="X",
                position=[0.0, 0.0],
                progress=5.0,
                departure_blockers=[],
                departure_action="",
            ),
            Waypoint(
                name="Y",
                position=[1.0, 0.0],
                progress=6.0,
                departure_blockers=[],
                departure_action="",
            ),
        ],
    )

    # Agent B expected
    blocker = TrafficDependency(
        name="A",
        plan_id=plan_ids["A"],
        required_progress=6.0,
    )

    expected_b = Plan(
        plan_id=plan_ids["B"],
        start_time=None,
        workflow="",
        waypoints=[
            Waypoint(
                name="Z",
                position=[0.0, 1.0],
                progress=10.0,
                departure_blockers=[blocker],
                departure_action="",
            ),
            Waypoint(
                name="X",
                position=[0.0, 0.0],
                progress=11.0,
                departure_blockers=[],
                departure_action="",
            ),
        ],
    )

    generated_map = {p.plan_id: p for p in generated}

    assert generated_map[plan_ids["A"]] == expected_a
    assert generated_map[plan_ids["B"]] == expected_b
