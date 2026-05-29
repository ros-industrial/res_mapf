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

AStar search

author: Ashwin Bose (@atb033)

"""

from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from res_mapf_planning.cbs.cbs import Environment, State


class AStar:
    def __init__(self, env: "Environment") -> None:
        self.env = env

    def reconstruct_path(
        self, came_from: dict["State", "State"], current: "State"
    ) -> list["State"]:
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def search(self, agent_name: str) -> Optional[list["State"]]:
        """
        low level search
        """
        initial_state = self.env.agent_dict[agent_name]["start"]
        step_cost = 1

        closed_set: set["State"] = set()
        open_set = {initial_state}

        came_from: dict["State", "State"] = {}

        g_score: dict["State", float] = {}
        g_score[initial_state] = 0

        f_score: dict["State", float] = {}

        f_score[initial_state] = self.env.admissible_heuristic(
            initial_state, agent_name
        )

        while open_set:
            temp_dict = {
                open_item: f_score.setdefault(open_item, float("inf"))
                for open_item in open_set
            }
            current = min(temp_dict, key=lambda open_item: temp_dict[open_item])

            if self.env.is_at_goal(current, agent_name):
                conflict = any(
                    vc.time >= current.time and vc.location == current.location
                    for vc in self.env.constraints.vertex_constraints
                )
                if not conflict:
                    return self.reconstruct_path(came_from, current)

            open_set -= {current}
            closed_set |= {current}

            neighbor_list = self.env.get_neighbors(current)

            for neighbor in neighbor_list:
                if neighbor in closed_set:
                    continue

                tentative_g_score = (
                    g_score.setdefault(current, float("inf")) + step_cost
                )

                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):
                    continue

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self.env.admissible_heuristic(
                    neighbor, agent_name
                )
        return None
