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
from dataclasses import dataclass
from typing import Optional

from res_mapf_planning.traffic_dependencies.models.plan import Plan
from res_mapf_planning.traffic_dependencies.models.plan_id import PlanId
from res_mapf_planning.traffic_dependencies.models.traffic_dependency import (
    TrafficDependency,
)
from res_mapf_planning.traffic_dependencies.models.waypoint import Waypoint
from res_plan_server.models.committed_location import CommittedLocation

logger = logging.getLogger("dependency_manager")


@dataclass
class RobotPlanState:
    robot_id: str
    plan: Plan
    current_waypoint: int  # index of most-recently-reached waypoiint
    latest_enqueued: int  # index of last enqueued waypoint


@dataclass
class CommitCut:
    committed_locations: dict[str, CommittedLocation]
    stationary_robots: set[str]


class DependencyManager:
    """
    Tracks each robot's plan, checks for traffic dependencies and releases waypoints.
    Computes commit cut.
    """

    def __init__(self) -> None:
        self._robots: dict[str, RobotPlanState] = {}

        # Track Plan IDs (destination session + plan version) that have been completed.
        # Blockers referencing completed plan IDs are considered satisfied.
        self._completed_plan_ids: set[PlanId] = set()  # TODO clean up

        self._cut_indices: dict[str, int] | None = None

    def set_plan(self, robot_id: str, plan: Plan) -> None:
        """
        Set a robot's plan, replacing the previous one.
        Waypoint 0 is the start waypoint.
        """
        self._robots[robot_id] = RobotPlanState(
            robot_id=robot_id,
            plan=plan,
            current_waypoint=0,
            latest_enqueued=0,
        )

    def update_plan_after_cut(self, robot_id: str, new_plan: Plan) -> None:
        """
        Retains waypoints before the commit cut.
        Replace waypoints after cut with new_plan's waypoints.
        Updates plan_id.
        Clear cut index.
        """
        state = self._robots[robot_id]
        old_plan_id = state.plan.plan_id

        if self._cut_indices and robot_id in self._cut_indices:
            cut_idx = self._cut_indices.pop(robot_id)  # clear stored commit cut
        else:
            cut_idx = state.latest_enqueued

        retained = state.plan.waypoints[: cut_idx + 1]
        new_waypoints = list(new_plan.waypoints)
        if new_waypoints[0].name == retained[-1].name:
            # Transfer blockers before removing
            retained[-1].departure_blockers.extend(new_waypoints[0].departure_blockers)
            new_waypoints = new_waypoints[1:]

        state.plan.waypoints = retained + new_waypoints
        state.plan.plan_id = new_plan.plan_id

        for i, wp in enumerate(state.plan.waypoints):
            logger.debug(
                "Robot %s wp[%d] name=%s progress=%.1f blockers=%s",
                robot_id,
                i,
                wp.name,
                wp.progress,
                [
                    (b.name, b.required_progress, b.plan_id)
                    for b in wp.departure_blockers
                ],
            )

        if old_plan_id is not None:
            self._restamp_blockers(old_plan_id, new_plan.plan_id)

    def _restamp_blockers(self, old_plan_id: PlanId, new_plan_id: PlanId) -> None:
        """
        Update all references to old_plan_id with new_plan_id.
        """
        for state in self._robots.values():
            for wp in state.plan.waypoints:
                wp.departure_blockers = [
                    TrafficDependency(
                        name=blocker.name,
                        plan_id=new_plan_id,
                        required_progress=blocker.required_progress,
                    )
                    if blocker.plan_id == old_plan_id
                    else blocker
                    for blocker in wp.departure_blockers
                ]

    def get_valid_waypoints_and_advance(
        self, robot_id: str, max_enqueued: int = 2
    ) -> list[tuple[int, Waypoint]]:
        """Get waypoints that can be enqueued, and marks them as enqueued.
        A waypoint can be enqueued safely when its preceding waypoint has its departure blockers (due to other robots) satisfied,
        and is included in the commit.

        Args:
            robot_id (str): _description_
            max_enqueued (int, optional): Max number of waypoints for a robot to have enqueued. Defaults to 2.

        Returns:
            list[tuple[int, Waypoint]]: _description_
        """
        state = self._robots.get(robot_id)
        if state is None or state.plan is None:
            return []

        waypoints = state.plan.waypoints

        # Don't enqueue beyond the commit
        upper_bound = len(waypoints) - 1
        if self._cut_indices and robot_id in self._cut_indices:
            upper_bound = self._cut_indices[robot_id]

        valid = []

        i = state.latest_enqueued

        logger.debug(
            "Robot %s current_waypoint %d latest_enqueued %d",
            robot_id,
            state.current_waypoint,
            state.latest_enqueued,
        )

        while i < upper_bound:
            # Check if robot already has max enqueued waypoints
            if i + 1 - state.current_waypoint > max_enqueued:
                logger.debug(
                    "Robot %s max waypoints already enqueued, not enqueuing waypoint %d",
                    robot_id,
                    (i + 1),
                )
                break
            # Check the current waypoint's departure blockers
            if not self._all_blockers_satisfied(waypoints[i].departure_blockers):
                break

            # Enqueue the next waypoint since we can depart from this waypoint
            valid.append((i + 1, waypoints[i + 1]))
            logger.debug("Robot %s Waypoint %d to be enqueued.", robot_id, (i + 1))

            i += 1
            state.latest_enqueued = i

        return valid

    def _all_blockers_satisfied(self, blockers: list[TrafficDependency]) -> bool:
        for blocker in blockers:
            if blocker.plan_id in self._completed_plan_ids:
                logger.debug("blocker %s is completed", blocker)
                continue

            state = self._robots.get(blocker.name)

            # Plan might not have arrived yet
            if state is None or state.plan is None:
                logger.debug(
                    "blocker %s: robot %s has no plan yet", blocker, blocker.name
                )

                return False
            if state.plan.plan_id != blocker.plan_id:
                logger.debug(
                    "blocker %s: robot %s on different plan", blocker, blocker.name
                )
                return False

            current_progress = state.plan.waypoints[state.current_waypoint].progress
            if current_progress < blocker.required_progress:
                logger.debug(
                    "blocker %s has not progressed past %.2f",
                    blocker.name,
                    blocker.required_progress,
                )
                return False

        return True

    def update_progress(self, robot_id: str, reached_waypoint: int) -> None:
        """This must be called when a waypoint is reached."""
        state = self._robots.get(robot_id)
        if state is None:
            logger.debug("update_progress: No plan state for %s", robot_id)
            return
        state.current_waypoint = reached_waypoint

    def compute_commit_cut(self) -> CommitCut:
        """
        Computes committed locations without truncating current plans.
        Returns committed waypoints and stationary robots.
        Ensures that the last-enqueued vertex is included.
        """

        cut_indices = self._compute_commit_cut()
        committed_locations = self._build_committed_locations(cut_indices)
        # Check which agents have completed all waypoints in this plan.
        stationary_agents = self._build_stationary_agents()
        self._cut_indices = cut_indices
        return CommitCut(committed_locations, stationary_agents)

    def _compute_commit_cut(self) -> dict[str, int]:
        """
        Computes the commit cut, beginning from what has been enqueued.
        """
        if not self._robots:
            logger.warning("No robots")
            return {}

        commit_cut = {
            robot_id: state.latest_enqueued
            if state.latest_enqueued is not None
            else state.current_waypoint
            for robot_id, state in self._robots.items()
        }
        logger.debug("commit cut: %s", str(commit_cut))
        previous_extended: set[str] = set()
        changed = True

        # Extend the commit cut to satisfy the departure blockers of all waypoints included in the committed waypoints
        while changed:  # Extending the commit cut may result in new departure blockers that require further extension of the commit cut
            changed = False
            current_extended: set[str] = set()

            for robot_id, state in self._robots.items():
                for i in range(state.current_waypoint, commit_cut[robot_id] + 1):
                    for blocker in state.plan.waypoints[i].departure_blockers:
                        required_idx = self._extend_commit_cut_for(blocker)

                        # Extend the commit cut to match this blocker.
                        # Track extensions to avoid cycles
                        if (
                            required_idx is not None
                            and blocker.name in commit_cut
                            and required_idx > commit_cut[blocker.name]
                        ):
                            commit_cut[blocker.name] = required_idx
                            current_extended.add(blocker.name)
                            changed = True

            if current_extended and current_extended == previous_extended:
                logger.error(
                    "Cycle found between robots: %s",
                    current_extended,
                )
                break

            previous_extended = current_extended

        return commit_cut

    def _extend_commit_cut_for(
        self, departure_blocker: TrafficDependency
    ) -> Optional[int]:
        """Finds the index of the earliest waypoint in this robot's plan that will satisfy the progress in the departure blocker.

        Args:
            departure_blocker (TrafficDependency):

        Returns:
            Optional[int]: _description_
        """
        if departure_blocker.plan_id in self._completed_plan_ids:
            return None

        state = self._robots.get(departure_blocker.name)

        assert state is not None, (
            f"Blocker references unknown robot {departure_blocker.name}"
        )

        assert state.plan.plan_id == departure_blocker.plan_id, (
            f"Departure blocker's plan id is {departure_blocker.plan_id} but "
            f"robot {departure_blocker.name} is on {state.plan.plan_id}."
        )

        for i, wp in enumerate(state.plan.waypoints):
            # Return the index of first waypoint with progress exceeding the blocker's progress
            if wp.progress >= departure_blocker.required_progress:
                return i
        logger.warning(
            "No waypoint matches required_progress %.2f for robot %s. Plan's progress values may have errors.",
            departure_blocker.required_progress,
            departure_blocker.name,
        )
        return len(state.plan.waypoints) - 1

    def _build_committed_locations(
        self, commit_cut: dict[str, int]
    ) -> dict[str, CommittedLocation]:
        result = {}

        for robot_id, cut_idx in commit_cut.items():
            state = self._robots.get(robot_id)
            if state is None:
                continue

            wp = state.plan.waypoints[cut_idx]

            result[robot_id] = CommittedLocation(
                robot_id=robot_id,
                location=wp.name,
                task_id=str(state.plan.plan_id) if state.plan.plan_id else None,
                waypoint_index=cut_idx,
                progress=wp.progress,
            )
        return result

    def _build_stationary_agents(self) -> set[str]:
        result = set()
        for robot_id, state in self._robots.items():
            # Robot is stationary if it has reached the last waypoint in its plan
            if state.current_waypoint >= len(state.plan.waypoints) - 1:
                result.add(robot_id)
        return result

    def has_robot(self, robot_id: str) -> bool:
        return robot_id in self._robots

    def get_plan_state(self, robot_id: str) -> Optional[RobotPlanState]:
        return self._robots.get(robot_id)

    def get_waypoint(self, robot_id: str, index: int) -> Optional[Waypoint]:
        state = self._robots.get(robot_id)
        if state is None or index < 0 or index >= len(state.plan.waypoints):
            return None
        return state.plan.waypoints[index]

    def get_task_id(self, robot_id: str) -> str | None:
        state = self._robots.get(robot_id)
        if state is None or state.plan.plan_id is None:
            return None
        return str(state.plan.plan_id)

    def get_robots(self) -> list[str]:
        return list(self._robots.keys())

    def is_complete(self, robot_id: str) -> bool:
        """True if the robot has reached its final waypoint."""
        state = self._robots.get(robot_id)
        if state is None:
            return False
        return state.current_waypoint >= len(state.plan.waypoints) - 1

    def on_plan_complete(self, robot_id: str) -> None:
        state = self._robots.get(robot_id)
        if state is not None and state.plan.plan_id is not None:
            self._completed_plan_ids.add(state.plan.plan_id)

    def on_plan_failed(self, robot_id: str) -> None:
        """Clears the robot's plan. Does not mark as completed. Other robots
        blocked on this plan will remain blocked.
        """
        state = self._robots.get(robot_id)
        if state is not None and state.plan.plan_id is not None:
            state.plan = None
