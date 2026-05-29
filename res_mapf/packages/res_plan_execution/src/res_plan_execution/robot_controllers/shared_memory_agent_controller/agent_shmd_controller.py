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

import json
import logging
import time
from threading import Thread
from typing import List

from res_mapf_planning.mapf_solve.mapf_solver_base import Location
from res_plan_execution.robot_controllers.base_robot_controller import (
    BaseRobotController,
    WaypointWithCallback,
)
from shared_memory_dict import SharedMemoryDict

logger = logging.getLogger("SharedMemoryAgentController")


class SharedMemoryAgentController(BaseRobotController):
    """Implements BaseRobotController interface to forward enqueued Actions to individual agents."""

    def __init__(self) -> None:
        self._shutdown = False

        self.robots: dict[str, SharedMemoryAgent] = {}
        self._shutdown = False

    def enqueue(
        self, robot_id: str, waypoints_with_callbacks: List[WaypointWithCallback]
    ) -> None:
        """
        Get this agent to enqueue actions for execution

        Args:
            actions_with_callbacks (WaypointWithCallback): List of WaypointWithCallback objects
        """

        if not waypoints_with_callbacks:
            logger.warning(f"Enqueued() was called with no waypoints for {robot_id}")
            return
        if robot_id not in self.robots:
            logger.info(f"Creating new agent {robot_id}.")
            self.robots[robot_id] = SharedMemoryAgent(robot_id)
        self.robots[robot_id].enqueue(waypoints_with_callbacks)

    def shutdown(self, interrupted: bool = False) -> None:
        if interrupted:
            logger.info("SharedMemoryAgentController was interrupted.")
        self._shutdown = True
        logger.info("SharedMemoryAgentController shutting down.")
        for robot in self.robots.values():
            robot.shutdown(interrupted)


class SharedMemoryAgent:
    def __init__(self, robot_id: str) -> None:
        self.robot_id = robot_id

        # Configs
        self._shmd = SharedMemoryDict(name=robot_id, size=8192)

        self._pending: list[tuple[int, WaypointWithCallback]] = []
        self._step_counter = 0
        self._shutdown = False

        self.begin_status_checking()

    def begin_status_checking(self) -> None:
        def _fn() -> None:
            while not self._shutdown:
                time.sleep(0.5)

                try:
                    res = self._shmd.get("action_completion_feedback")
                    # logger.debug(f"action_completion_feedback: {res}")

                    if not res:
                        continue

                    results = json.loads(res)
                    if not self._pending:
                        continue

                    current_idx, current_wp = self._pending[0]

                    for result in results:
                        action = result.get("action", {})
                        if (
                            action.get("step_index") == current_idx
                            and result.get("state") == "ActionState.COMPLETED"
                        ):
                            logger.info(
                                "%s completed step %d (%s)",
                                self.robot_id,
                                current_idx,
                                current_wp.location,
                            )
                            current_wp.on_reached()
                            self._pending.pop(0)
                            break

                except (KeyError, json.JSONDecodeError):
                    logger.info("Invalid message from pybullet result")

            self._cleanup()

        t = Thread(target=_fn)
        t.start()

    def enqueue(self, waypoints_w_cb: List[WaypointWithCallback]) -> None:
        """
        Get this agent to enqueue actions for execution

        Args:
            actions_with_callbacks (WaypointWithCallback): List of WaypointWithCallback objects
        """

        if not waypoints_w_cb:
            logger.info(
                f"Warning: enqueued() was called with no actions for {self.robot_id}"
            )
            return
        for wp_w_cb in waypoints_w_cb:
            self._pending.append((self._step_counter, wp_w_cb))
            logger.info(
                "Enqueued step %d (%s) for %s",
                self._step_counter,
                wp_w_cb.location,
                self.robot_id,
            )
            self._step_counter += 1

        self._publish()

    def _location_to_str(self, location: Location) -> str:
        if location.is_named():
            return str(location.name)
        return f"{location.x},{location.y}"

    def _publish(self) -> None:
        # TODO: cleanup
        commands = [
            {
                "state": "ActionState.QUEUED",
                "action": {
                    "step_index": step_idx,
                    "location_end": self._location_to_str(wp.location),
                },
            }
            for step_idx, wp in self._pending
        ]
        try:
            self._shmd["command"] = json.dumps(commands)
            logger.info(f"Publishing command: {commands}")

        except ValueError:
            logger.error(
                "Increase the size of the SharedMemoryDict for all users of the dict so that it can hold the command."
            )
            raise

    # Differentiate between completion and errors.
    def shutdown(self, interrupted: bool = False) -> None:
        if interrupted:
            logger.info(f"{self.robot_id} was interrupted.")
        self._shutdown = True
        logger.info(f"{self.robot_id} shutting down.")

    def _cleanup(self) -> None:
        self._shmd.shm.close()
        self._shmd.shm.unlink()
        del self._shmd
