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

from typing import Any, Dict, Optional, Sequence, Tuple
import json
from enum import Enum

from shared_memory_dict import SharedMemoryDict  # type: ignore


class AgentState(Enum):
    MOVING = (0, 0, 1)  # blue
    WAITING = (255, 165, 0)  # orange
    COMPLETED = (0, 1, 0)  # green
    UNKNOWN = (1, 0, 0)  # red
    REACHED_WP = (1, 1, 11)
    MOVING_R = (0, 0, 12)
    ROTATING = (0, 0, 255)


class Agent:
    """Used by simulation for internal tracking"""

    def __init__(self, id: int):
        self.id = id
        self.last_vel_cmd = 0.0
        self._reached_goal = False
        self.agent_state = AgentState.WAITING

    def reached_goal(self, reached: bool) -> None:
        self._reached_goal = reached

    def is_reached_goal(self) -> bool:
        return self._reached_goal


class AgentCommandInterface:
    """
    This class provides functions for agents to receive commands and indicate completion over the SharedMemoryDict.
    TODO: Interface
    """

    def __init__(
        self, name: str, vertices_map: Optional[Dict[str, Sequence[float]]] = None
    ):
        self.name = name
        self.current_command: Any = None
        self.shmd = SharedMemoryDict(name=name, size=2048)
        # TODO cleanup using # self.shmd.shm.close()
        self.agent_most_recent_wp = -1

        self._init_flag = False

        if vertices_map:
            self.vertices_map = vertices_map

    ### Functions for use by external requester

    def add_command_to_queue(self, x: float, y: float, cmd_id: int) -> None:
        raise NotImplementedError

    def reset_destination(self, x: float, y: float, cmd_id: int) -> None:
        raise NotImplementedError

    def get_all_commands(self) -> None:
        raise NotImplementedError

    def is_completed(self, cmd_id: int) -> None:
        raise NotImplementedError

    def convert(self, step_to: str) -> Sequence[float]:
        # Returns tuple of target coordinates
        # Looks up vertices if not in x,y format

        if "," in step_to:  # x,y format
            targ = tuple(step_to.split(","))
            targ_float = tuple([float(n) for n in targ])
            return targ_float

        targ_str = step_to
        targ_float = tuple(self.vertices_map[targ_str])

        return targ_float

    def command_str_to_target(
        self, command: str
    ) -> Tuple[Optional[str], Optional[int], Optional[str], bool]:
        """Parses the string containing the command
        Args:
            command (str): raw string for the command

        Returns:
            _type_: _description_
        """
        step_to = None
        target_step_index = None
        next_step_to = None
        is_goal = False

        command = json.loads(command)
        self.current_command = command
        for idx, cmd in enumerate(command):
            if (
                cmd["state"] == "ActionState.QUEUED"
                and cmd["action"]["step_index"] > self.agent_most_recent_wp
            ):
                target_step_index = cmd["action"]["step_index"]
                step_to = cmd["action"]["location_end"]

                if idx == len(command) - 1:
                    is_goal = True
                else:
                    next_cmd = command[idx + 1]
                    if next_cmd["state"] == "ActionState.QUEUED":
                        next_step_to = next_cmd["action"]["location_end"]
                break
        if target_step_index is None:
            return None, None, None, False
        return step_to, target_step_index, next_step_to, is_goal

    def get_target(
        self,
    ) -> Tuple[
        Optional[Sequence[float]], Optional[int], Optional[Sequence[float]], bool
    ]:
        """
        TODO: Clean up return value
        Returns:
            (x, y), target_id, (next_x, next_y), is_goal: Tuple containing x and y of target location for agent, and if it is the goal.
        """

        if "command" not in self.shmd:
            return None, None, None, False
        target = None
        target_id = None
        next = None
        is_goal = False

        step_to, target_id, next_step_to, is_goal = self.command_str_to_target(
            self.shmd["command"]
        )
        if step_to:
            target = self.convert(step_to)
        if next_step_to:
            next = self.convert(next_step_to)
        # print("Found target:", target, " next target:", next)
        return target, target_id, next, is_goal

    def completed(self, waypoint_idx: int) -> None:
        """Indicate completion of waypoint"""
        self.agent_most_recent_wp = waypoint_idx

        for idx, cmd in enumerate(self.current_command):
            if (
                cmd["state"] == "ActionState.QUEUED"
                and cmd["action"]["step_index"] == waypoint_idx
            ):
                cmd["state"] = "ActionState.COMPLETED"
                self.shmd["action_completion_feedback"] = json.dumps(
                    [cmd]
                )  # list is expected
