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

"""Solve the small graph shown in the MAPF visualiser.

Run from the res_mapf repository root:

    uv run python examples/example_mapf_cbs_search.py
"""

from typing import Sequence, TypedDict

from res_mapf_planning.cbs.cbs import CBS, AgentContext, Environment, State
from utils.node_visualiser import render_timeline


class Node(TypedDict):
    id: str
    x: float
    y: float


# Only the nodes visible in the screenshot. The ids are the MAPF grid positions;
# x and y are their physical AMAV-X map coordinates from the LIF clone.
NODES: list[Node] = [
    {"id": "1,5", "x": -10.0, "y": 7.5},
    {"id": "2,5", "x": -6.0, "y": 7.5},
    {"id": "3,5", "x": -3.0, "y": 7.5},
    {"id": "4,5", "x": 0.0, "y": 7.5},
    {"id": "5,5", "x": 1.9, "y": 7.5},
    {"id": "2,4", "x": -6.0, "y": 3.0},
    {"id": "3,4", "x": -1.5, "y": 5.0},
    {"id": "4,4", "x": 0.0, "y": 5.0},
    {"id": "5,4", "x": 2.0, "y": 5.0},
    {"id": "3,3", "x": -1.5, "y": 3.0},
    {"id": "4,3", "x": 0.0, "y": 3.0},
]

# LIF edges are treated as bidirectional for this example.
EDGES: list[tuple[str, str]] = [
    ("1,5", "2,5"),
    ("2,5", "3,5"),
    ("3,5", "4,5"),
    ("4,5", "5,5"),
    ("2,5", "2,4"),
    ("4,5", "4,4"),
    ("4,4", "4,3"),
    ("3,3", "4,3"),
    ("3,4", "4,4"),
    ("5,4", "4,4"),
]


def grid_position(node_id: str) -> tuple[int, int]:
    """Convert a LIF node id such as ``4,5`` to the CBS grid position."""
    column, row = node_id.split(",")
    return int(column), int(row)


class GraphEnvironment(Environment):
    """Restrict the existing CBS solver to the nodes and edges above."""

    def __init__(self, agents: Sequence[AgentContext]) -> None:
        self.nodes = {grid_position(node["id"]) for node in NODES}
        self.edges = {
            move
            for start_id, end_id in EDGES
            for move in (
                (grid_position(start_id), grid_position(end_id)),
                (grid_position(end_id), grid_position(start_id)),
            )
        }
        dimension = [
            max(x for x, _ in self.nodes) + 1,
            max(y for _, y in self.nodes) + 1,
        ]
        super().__init__(dimension, agents, obstacles=[])

    def state_valid(self, state: State) -> bool:
        position = (state.location.x, state.location.y)
        return position in self.nodes and super().state_valid(state)

    def transition_valid(self, start: State, end: State) -> bool:
        move = (
            (start.location.x, start.location.y),
            (end.location.x, end.location.y),
        )
        return move in self.edges and super().transition_valid(start, end)


def main() -> None:
    # These paths put the robots at the positions shown in the screenshot at t=2:s
    agents: list[AgentContext] = [
        {
            "name": "robot-a",
            "start": grid_position("2,5"),
            "goal": grid_position("4,5"),
        },
        {
            "name": "robot-b",
            "start": grid_position("5,5"),
            "goal": grid_position("4,3"),
        },
    ]

    solution = CBS(GraphEnvironment(agents)).search()
    if not solution:
        raise RuntimeError("CBS could not find a solution")

    for robot_id, path in solution.items():
        steps = [f"{step['x']},{step['y']}" for step in path]
        print(f"{robot_id}: {' -> '.join(steps)}")

    print()
    print(render_timeline(NODES, EDGES, solution))


if __name__ == "__main__":
    main()
