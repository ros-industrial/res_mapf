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
class PlanLedgerEntry:
    """Each entry in a robot's ledger is a Plan and the position of its first waypoint in the robot's
    waypoints array."""
    plan_id: PlanId
    array_offset: int


@dataclass
class RobotPlanLedger:
    """Hold all Plans and waypoints this robot is currently tracking.
    """    
    robot_id: str
    waypoints: list[Waypoint]   # across all Plans
    plan_entries: list[PlanLedgerEntry]
    current_waypoint: int = 0   # index of most-recently-reached waypoint
    latest_enqueued: int = 0 # index of last enqueued waypoint
    cut_index: int | None = None
    execution_failed: bool = False


@dataclass
class CommitCut:
    committed_locations: dict[str, CommittedLocation]
    stationary_robots: set[str]
    possible_obstacle_locations: list[str]     # Waypoints that were dispatched but not reached. Robot might be on/between any. Must be treated as obstacles until resolved.


@dataclass(frozen=True)
class DispatchedWaypoint:
    """Returned by get_valid_waypoints_and_advance. To be passed back into update_progress."""
    plan_id: PlanId
    waypoint: Waypoint
    index: int  # position of this waypoint within its plan


@dataclass(frozen=True)
class ProgressReport:
    """Returned by get_progress. reached_index/target_index are relative to
    the robot's current plan_id."""
    plan_id: PlanId
    reached_index: int
    target_index: int


class DependencyManager:
    """
    Tracks each robot's plan, checks for traffic dependencies and releases waypoints.
    Computes commit cut.
    """

    def __init__(self) -> None:
        self._robots: dict[str, RobotPlanLedger] = {}

        # Track Plan IDs (destination session + plan version) that have been completed.
        # Blockers referencing completed plan IDs are considered satisfied.
        self._completed_plan_ids: set[PlanId] = set()  # TODO clean up


    def set_plan(self, robot_id: str, plan: Plan) -> None:
        """
        Set a robot's plan, replacing the previous one and discarding history.
        Waypoint 0 is the start waypoint.
        """
        self._robots[robot_id] = RobotPlanLedger(
            robot_id=robot_id,
            waypoints=list(plan.waypoints),
            plan_entries=[PlanLedgerEntry(plan_id=plan.plan_id, array_offset=0)],
            current_waypoint=0,
            latest_enqueued=0,
            cut_index=None,
        )

    def update_plan_after_cut(self, robot_id: str, new_plan: Plan) -> None:
        """
        Retains waypoints up to and including the commit cut.
        Appends new_plan's waypoints as a new ledger entry.
        Clears cut index.
        """
        ledger = self._robots[robot_id]

        cut_idx = (
            ledger.cut_index if ledger.cut_index is not None else ledger.latest_enqueued
        )
        ledger.cut_index = None  # clear the previous cut

        retained = ledger.waypoints[: cut_idx + 1]
        new_waypoints = list(new_plan.waypoints)

        # new_plan's waypoint 0 is expected to be the the same location as the previous plan's last-committed.
        # otherwise right after it.
        array_offset = cut_idx + 1
        if new_waypoints[0].name == retained[-1].name:
            # Transfer blockers before removing the duplicate.
            retained[-1].departure_blockers.extend(new_waypoints[0].departure_blockers)
            new_waypoints = new_waypoints[1:]
            array_offset = cut_idx

        ledger.waypoints = retained + new_waypoints

        # A plan that does not start before new_plan has had all of its own
        # waypoints replaced, remove it from the ledger without marking as complete.
        ledger.plan_entries = [
            entry for entry in ledger.plan_entries if entry.array_offset < array_offset
        ]
        ledger.plan_entries.append(
            PlanLedgerEntry(plan_id=new_plan.plan_id, array_offset=array_offset)
        )

        # A new plan resolves a failed execution.
        ledger.execution_failed = False

        for i, wp in enumerate(ledger.waypoints):
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

        self._drop_finished_plans(ledger)

    def get_valid_waypoints_and_advance(
        self, robot_id: str, max_enqueued: int = 2
    ) -> list[DispatchedWaypoint]:
        """Get waypoints that can be dispatched, and marks them as enqueued.
        A waypoint can be enqueued safely when its preceding waypoint has its departure blockers (due to other robots) satisfied,
        and is included in the commit.
        First waypoint of initial Plan is the start location and does not need to be enqueued.

        Args:
            robot_id (str): _description_
            max_enqueued (int, optional): Max number of waypoints for a robot to have enqueued. Defaults to 2.

        Returns:
            list[DispatchedWaypoint]: waypoints newly enqueued, to pass to the robot controller.
        """
        ledger = self._robots.get(robot_id)
        if ledger is None or ledger.execution_failed:
            return []

        waypoints = ledger.waypoints

        # Don't enqueue beyond the commit.
        # This function might be called after a commit cut is determined, but before a new plan arrives.
        upper_bound = len(waypoints) - 1
        if ledger.cut_index is not None:
            upper_bound = ledger.cut_index   # inclusive upper bound

        valid: list[DispatchedWaypoint] = []

        i = ledger.latest_enqueued

        logger.debug(
            "Robot %s current_waypoint %d latest_enqueued %d",
            robot_id,
            ledger.current_waypoint,
            ledger.latest_enqueued,
        )

        while i < upper_bound:
            # Check if robot already has max enqueued waypoints
            if i + 1 - ledger.current_waypoint > max_enqueued:
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
            i += 1
            ledger.latest_enqueued = i
            entry = self._entry_for_position(ledger, i)
            valid.append(
                DispatchedWaypoint(
                    plan_id=entry.plan_id,
                    waypoint=waypoints[i],
                    index=i - entry.array_offset,
                )
            )
            logger.debug("Robot %s Waypoint %d to be enqueued.", robot_id, i)

        return valid

    def _all_blockers_satisfied(self, blockers: list[TrafficDependency]) -> bool:
        for blocker in blockers:
            if blocker.plan_id in self._completed_plan_ids:
                logger.debug("blocker %s is completed", blocker)
                continue

            ledger = self._robots.get(blocker.name)

            # Plan might not have arrived yet
            if ledger is None:
                logger.debug(
                    "blocker %s: robot %s has no plan yet", blocker, blocker.name
                )

                return False
            if self._find_plan_entry(ledger, blocker.plan_id) is None:
                logger.debug(
                    "blocker %s: robot %s on different plan", blocker, blocker.name
                )
                return False

            current_progress = ledger.waypoints[ledger.current_waypoint].progress
            if current_progress < blocker.required_progress:
                logger.debug(
                    "blocker %s has not progressed past %.2f",
                    blocker.name,
                    blocker.required_progress,
                )
                return False

        return True

    def update_progress(self, robot_id: str, reached: DispatchedWaypoint) -> bool:
        """This must be called when a waypoint is reached.
        Returns False if this update was ignored.
        """
        ledger = self._robots.get(robot_id)
        if ledger is None:
            logger.debug("update_progress: No plan ledger for %s", robot_id)
            return False

        entry = self._find_plan_entry(ledger, reached.plan_id)
        if entry is None:
            logger.warning(
                "Robot %s: ignoring waypoint %s - plan %s is not part of its current ledger",
                robot_id, reached.waypoint.name, reached.plan_id,
            )
            return False

        position = entry.array_offset + reached.index
        if not (ledger.current_waypoint < position <= ledger.latest_enqueued):
            logger.warning(
                "Robot %s: ignoring out-of-range waypoint %s (position %d not in [%d, %d])",
                robot_id, reached.waypoint.name, position, ledger.current_waypoint, ledger.latest_enqueued,
            )
            return False

        ledger.current_waypoint = position
        self._drop_finished_plans(ledger)
        return True

    @staticmethod
    def _find_plan_entry(
        ledger: RobotPlanLedger, plan_id: PlanId
    ) -> PlanLedgerEntry | None:
        for entry in ledger.plan_entries:
            if entry.plan_id == plan_id:
                return entry
        return None

    def on_enqueue_rejected(self, robot_id: str) -> bool:
        """The controller rejected an enqueue. If the robot never confirmed reaching anything on
        this ledger, drop it entirely so the next plan for this robot starts
        fresh via set_plan, rather than being spliced onto a ledger that never
        actually ran."""
        ledger = self._robots.get(robot_id)
        if ledger is None:
            return False
        if ledger.current_waypoint == 0:
            del self._robots[robot_id]
        else:
            ledger.execution_failed = True
        return True

    def _drop_finished_plans(self, ledger: RobotPlanLedger) -> None:
        """Clean up completed plans that the robot has moved past from the ledger.
        Track as completed for other robots' blockers.
        """
        while (
            len(ledger.plan_entries) > 1
            and ledger.plan_entries[1].array_offset <= ledger.current_waypoint
        ):
            self._completed_plan_ids.add(ledger.plan_entries.pop(0).plan_id)

        dropped = ledger.plan_entries[0].array_offset
        if dropped == 0:
            return
        del ledger.waypoints[:dropped]
        ledger.current_waypoint -= dropped
        ledger.latest_enqueued -= dropped
        if ledger.cut_index is not None:
            ledger.cut_index -= dropped
        for entry in ledger.plan_entries:
            entry.array_offset -= dropped


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
        possible_obstacle_locations = self._build_failed_robot_obstacles()
        for robot_id, idx in cut_indices.items():
            self._robots[robot_id].cut_index = idx
        return CommitCut(committed_locations, stationary_agents, possible_obstacle_locations)

    def _compute_commit_cut(self) -> dict[str, int]:
        """
        Computes the commit cut, beginning from what has been enqueued.
        """
        if not self._robots:
            logger.warning("No robots")
            return {}

        commit_cut = {
            robot_id: ledger.latest_enqueued
            if ledger.latest_enqueued is not None
            else ledger.current_waypoint
            for robot_id, ledger in self._robots.items()
        }
        logger.debug("commit cut: %s", str(commit_cut))
        previous_extended: set[str] = set()
        changed = True

        # Extend the commit cut to satisfy the departure blockers of all waypoints included in the committed waypoints
        while changed:  # Extending the commit cut may result in new departure blockers that require further extension of the commit cut
            changed = False
            current_extended: set[str] = set()

            for robot_id, ledger in self._robots.items():
                for i in range(ledger.current_waypoint, commit_cut[robot_id] + 1):
                    for blocker in ledger.waypoints[i].departure_blockers:
                        required_idx = self._extend_commit_cut_for(blocker)

                        # Extend the commit cut to match this blocker.
                        # Track extensions to avoid cycles
                        if (
                            required_idx is not None
                            and blocker.name in commit_cut
                            and required_idx > commit_cut[blocker.name]
                            and not self._robots[blocker.name].execution_failed
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

        ledger = self._robots.get(departure_blocker.name)

        assert ledger is not None, (
            f"Blocker references unknown robot {departure_blocker.name}"
        )

        assert self._find_plan_entry(ledger, departure_blocker.plan_id) is not None, (
            f"Departure blocker's plan id {departure_blocker.plan_id} is not in "
            f"robot {departure_blocker.name}'s plan history."
        )

        for i, wp in enumerate(ledger.waypoints):
            # Return the index of first waypoint with progress exceeding the blocker's progress
            if wp.progress >= departure_blocker.required_progress:
                return i
        logger.warning(
            "No waypoint matches required_progress %.2f for robot %s. Plan's progress values may have errors.",
            departure_blocker.required_progress,
            departure_blocker.name,
        )
        return len(ledger.waypoints) - 1

    @staticmethod
    def _entry_for_position(ledger: RobotPlanLedger, position: int) -> PlanLedgerEntry:
        """The plan that contributed the waypoint at position.
        A plan's waypoint 0 is the cut waypoint it was spliced onto, which
        belongs to the plan before it - so this must use <, not <=."""
        owner = ledger.plan_entries[0]
        for entry in ledger.plan_entries:
            if entry.array_offset < position:
                owner = entry
        return owner

    def _build_committed_locations(
        self, commit_cut: dict[str, int]
    ) -> dict[str, CommittedLocation]:
        result = {}

        for robot_id, cut_idx in commit_cut.items():
            ledger = self._robots.get(robot_id)
            if ledger is None:
                continue

            wp = ledger.waypoints[cut_idx]
            entry = self._entry_for_position(ledger, cut_idx)

            result[robot_id] = CommittedLocation(
                robot_id=robot_id,
                location=wp.name,
                task_id=str(entry.plan_id) if entry.plan_id else None,
                waypoint_index=cut_idx - entry.array_offset,
                progress=wp.progress,
            )
        return result

    def _build_stationary_agents(self) -> set[str]:
        result = set()
        for robot_id, ledger in self._robots.items():
            if ledger.execution_failed:
                # Stationary means the robot finished its plan and is parked
                # at its goal. Failed robots handled separately.
                continue
            # Robot is stationary if it has reached the last waypoint in its plan
            if ledger.current_waypoint >= len(ledger.waypoints) - 1:
                result.add(robot_id)
        return result

    def _build_failed_robot_obstacles(self) -> list[str]:
        """
        Waypoint names a failed robot may currently occupy: everything it was
        dispatched to (up to latest_enqueued) beyond its last confirmed
        waypoint (current_waypoint). Execution failure means we can no longer
        trust that it stopped exactly where it last reported.
        """
        locations = []
        for ledger in self._robots.values():
            if not ledger.execution_failed:
                continue
            for i in range(ledger.current_waypoint + 1, ledger.latest_enqueued + 1):
                locations.append(ledger.waypoints[i].name)
        return locations

    def has_robot(self, robot_id: str) -> bool:
        return robot_id in self._robots

    def get_ledger(self, robot_id: str) -> Optional[RobotPlanLedger]:
        return self._robots.get(robot_id)

    def get_current_destination_session(self, robot_id: str) -> str | None:
        """The stable task identifier for whichever plan the robot is
        currently on (owns current_waypoint) - not always the latest entry,
        since dispatch can be ahead of confirmed progress."""
        ledger = self._robots.get(robot_id)
        if ledger is None or not ledger.plan_entries:
            return None
        entry = self._entry_for_position(ledger, ledger.current_waypoint)
        return str(entry.plan_id.destination_session)

    def get_robots(self) -> list[str]:
        return list(self._robots.keys())

    def is_complete(self, robot_id: str) -> bool:
        """True if the robot has reached its final waypoint."""
        ledger = self._robots.get(robot_id)
        if ledger is None:
            return False
        return ledger.current_waypoint >= len(ledger.waypoints) - 1

    def on_plan_complete(self, robot_id: str, plan_id: PlanId) -> bool:
        ledger = self._robots.get(robot_id)
        if ledger is None:
            logger.warning("on_plan_complete called for unknown robot %s", robot_id)
            return False
        if not any(entry.plan_id == plan_id for entry in ledger.plan_entries):
            logger.warning(
                "on_plan_complete called for %s with plan_id %s not in its ledger",
                robot_id, plan_id,
            )
            return False
        self._completed_plan_ids.add(plan_id)
        return True

    def on_plan_failed(self, robot_id: str, plan_id: PlanId) -> bool:
        """If plan_id is this robot's active plan, mark plan execution as failed
        Returns False if the robot is on a different plan.
        Does not mark as completed - other robots blocked on this plan will remain blocked.
        Keeps the plan (rather than clearing it) so the robot's last confirmed
        location and progress remain known for the commit cut.
        """
        ledger = self._robots.get(robot_id)
        if ledger is None or not ledger.plan_entries:
            return False
        if ledger.plan_entries[-1].plan_id != plan_id:
            logger.debug(
                "Ignoring failure for %s: plan %s is no longer active", robot_id, plan_id
            )
            return False
        ledger.execution_failed = True
        return True

    def get_progress(self, robot_id: str) -> ProgressReport | None:
        """Progress relative to whichever plan owns current_waypoint - not
        always the latest entry, since dispatch can be ahead of confirmed
        progress. No clamp needed: entry.array_offset is always <= current_waypoint
        by definition of ownership, so the subtraction is never negative."""
        ledger = self._robots.get(robot_id)
        if ledger is None:
            return None
        entry = self._entry_for_position(ledger, ledger.current_waypoint)
        return ProgressReport(
            plan_id=entry.plan_id,
            reached_index=ledger.current_waypoint - entry.array_offset,
            target_index=ledger.latest_enqueued - entry.array_offset,
        )
