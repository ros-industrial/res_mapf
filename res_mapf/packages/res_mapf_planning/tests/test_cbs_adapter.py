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

from res_mapf_planning.mapf_solve.mapf_solver_base import Location, MAPFAgent
from res_mapf_planning.mapf_solve.models.models import SolverPlan
from res_mapf_planning.mapf_solve.solvers.cbs_adapter import CBSAdapter


class TestCBSAdapter:
    cbs_adapter = CBSAdapter()

    def test_solve(self) -> None:
        items = [
            MAPFAgent(
                task_id="test0",
                agent_id="0",
                start=Location(x=0, y=0),
                goal=Location(x=2, y=0),
            ),
            MAPFAgent(
                task_id="test1",
                agent_id="1",
                start=Location(x=2, y=0),
                goal=Location(x=0, y=0),
            ),
            MAPFAgent(
                task_id="test2",
                agent_id="2",
                start=Location(x=1, y=0),
                goal=Location(x=1, y=2),
            ),
            MAPFAgent(
                task_id="test3",
                agent_id="3",
                start=Location(x=4, y=0),
                goal=Location(x=4, y=4),
            ),
        ]

        plans = self.cbs_adapter.solve(items)

        assert all(isinstance(item, SolverPlan) for item in plans)
