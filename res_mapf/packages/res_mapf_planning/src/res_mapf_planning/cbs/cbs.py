# MIT License

# Copyright (c) 2019 Ashwin Bose

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""

Python implementation of Conflict-based search

author: Ashwin Bose (@atb033)

"""

from typing import Dict, List, Mapping, Optional, Sequence, Tuple, TypedDict
import argparse
from copy import deepcopy
from itertools import combinations
from math import fabs

import yaml
from res_mapf_planning.cbs.a_star import AStar
from res_mapf_planning.cbs.constraints import (
    Constraints,
    EdgeConstraint,
    VertexConstraint,
)


class Location(object):
    def __init__(self, x: int = -1, y: int = -1) -> None:
        self.x = x
        self.y = y

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Location):
            return NotImplemented
        return self.x == other.x and self.y == other.y

    def __str__(self) -> str:
        return str((self.x, self.y))


class State(object):
    def __init__(self, time: int, location: Location) -> None:
        self.time = time
        self.location = location

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, State):
            return NotImplemented
        return self.time == other.time and self.location == other.location

    def __hash__(self) -> int:
        return hash(str(self.time) + str(self.location.x) + str(self.location.y))

    def is_equal_except_time(self, state: "State") -> bool:
        return self.location == state.location

    def __str__(self) -> str:
        return str((self.time, self.location.x, self.location.y))


class Conflict(object):
    VERTEX = 1
    EDGE = 2

    def __init__(self) -> None:
        self.time = -1
        self.type = -1

        self.agent_1 = ""
        self.agent_2 = ""

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self) -> str:
        return (
            "("
            + str(self.time)
            + ", "
            + self.agent_1
            + ", "
            + self.agent_2
            + ", "
            + str(self.location_1)
            + ", "
            + str(self.location_2)
            + ")"
        )


class AgentContext(TypedDict):
    name: str
    start: Sequence[int]
    goal: Sequence[int]


class AgentEntry(TypedDict):
    start: State
    goal: State


class Environment(object):
    def __init__(
        self,
        dimension: Sequence[int],
        agents: Sequence[AgentContext],
        obstacles: Sequence[Tuple[int, int]],
    ) -> None:
        self.dimension = dimension
        self.obstacles = obstacles

        self.agents = agents
        self.agent_dict: Dict[str, AgentEntry] = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict: Dict[str, Constraints] = {}

        self.a_star = AStar(self)

    def get_neighbors(self, state: State) -> List[State]:
        neighbors: List[State] = []

        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y + 1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y - 1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x - 1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x + 1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors

    def get_first_conflict(
        self, solution: Dict[str, List[State]]
    ) -> Optional[Conflict]:
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t + 1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t + 1)

                if state_1a.is_equal_except_time(
                    state_2b
                ) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return None

    def create_constraints_from_conflict(
        self, conflict: Conflict
    ) -> Dict[str, Constraints]:
        constraint_dict: Dict[str, Constraints] = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(
                conflict.time, conflict.location_1, conflict.location_2
            )
            e_constraint2 = EdgeConstraint(
                conflict.time, conflict.location_2, conflict.location_1
            )

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def get_state(
        self, agent_name: str, solution: Dict[str, List[State]], t: int
    ) -> State:
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state: State) -> bool:
        return (
            state.location.x >= 0
            and state.location.x < self.dimension[0]
            and state.location.y >= 0
            and state.location.y < self.dimension[1]
            and VertexConstraint(state.time, state.location)
            not in self.constraints.vertex_constraints
            and (state.location.x, state.location.y) not in self.obstacles
        )

    def transition_valid(self, state_1: State, state_2: State) -> bool:
        return (
            EdgeConstraint(state_1.time, state_1.location, state_2.location)
            not in self.constraints.edge_constraints
        )

    def is_solution(self, agent_name: str) -> None:
        pass

    def admissible_heuristic(self, state: State, agent_name: str) -> float:
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(
            state.location.y - goal.location.y
        )

    def is_at_goal(self, state: State, agent_name: str) -> bool:
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self) -> None:
        for agent in self.agents:
            start_state = State(0, Location(agent["start"][0], agent["start"][1]))
            goal_state = State(0, Location(agent["goal"][0], agent["goal"][1]))

            entry: AgentEntry = {"start": start_state, "goal": goal_state}
            self.agent_dict[agent["name"]] = entry

    def compute_solution(self) -> Optional[Dict[str, List[State]]]:
        solution: Dict[str, List[State]] = {}
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return None
            solution.update({agent: local_solution})
        return solution

    def compute_solution_cost(self, solution: Mapping[str, Sequence[object]]) -> int:
        return sum([len(path) for path in solution.values()])


class HighLevelNode(object):
    def __init__(self) -> None:
        self.solution: Dict[str, List[State]] = {}
        self.constraint_dict: Dict[str, Constraints] = {}
        self.cost = 0

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, type(self)):
            return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self) -> int:
        return hash((self.cost))

    def __lt__(self, other: "HighLevelNode") -> bool:
        return self.cost < other.cost


class CBSPathState(TypedDict):
    t: int
    x: int
    y: int


CBSPath = Sequence[CBSPathState]
CBSPlan = Dict[str, CBSPath]


class CBS(object):
    def __init__(self, environment: Environment) -> None:
        self.env = environment
        self.open_set: set[HighLevelNode] = set()
        self.closed_set: set[HighLevelNode] = set()

    def search(self) -> CBSPlan:
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()
        solution = self.env.compute_solution()
        if not solution:
            return {}
        start.solution = solution
        start.cost = self.env.compute_solution_cost(start.solution)

        self.open_set |= {start}

        while self.open_set:
            P = min(self.open_set)
            self.open_set -= {P}
            self.closed_set |= {P}

            self.env.constraint_dict = P.constraint_dict
            conflict_dict = self.env.get_first_conflict(P.solution)
            if not conflict_dict:
                print("solution found")

                return self.generate_plan(P.solution)

            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                self.env.constraint_dict = new_node.constraint_dict
                solution = self.env.compute_solution()
                if not solution:
                    continue
                new_node.solution = solution
                new_node.cost = self.env.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}

        return {}

    def generate_plan(self, solution: Dict[str, List[State]]) -> CBSPlan:
        plan: CBSPlan = {}
        for agent, path in solution.items():
            path_dict_list: List[CBSPathState] = [
                {"t": state.time, "x": state.location.x, "y": state.location.y}
                for state in path
            ]
            plan[agent] = path_dict_list
        return plan


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("param", help="input file containing map and obstacles")
    parser.add_argument("output", help="output file with the schedule")
    args = parser.parse_args()

    # Read from input file
    with open(args.param, "r") as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = param["map"]["dimensions"]
    obstacles = param["map"]["obstacles"]
    agents = param["agents"]

    env = Environment(dimension, agents, obstacles)

    # Searching
    cbs = CBS(env)
    solution = cbs.search()
    if not solution:
        print(" Solution not found")
        return

    # Write to output file
    output: Dict[str, object] = dict()
    output["schedule"] = solution
    output["cost"] = env.compute_solution_cost(solution)
    with open(args.output, "w") as output_yaml:
        yaml.safe_dump(output, output_yaml)


if __name__ == "__main__":
    main()
