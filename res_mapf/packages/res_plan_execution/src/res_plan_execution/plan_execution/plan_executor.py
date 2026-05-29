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
import queue
import threading
from typing import Callable, Optional

from res_mapf_planning.mapf_solve.mapf_solver_base import Location
from res_mapf_planning.traffic_dependencies.models.plan import Plan
from res_plan_execution.plan_execution.dependency_manager import DependencyManager
from res_plan_execution.plan_execution.transport.executor_base_transport import (
    ExecutorBaseTransport,
)
from res_plan_execution.robot_controllers.base_robot_controller import (
    BaseRobotController,
    WaypointWithCallback,
)
from res_plan_server.task_status import TaskStatus, TaskStatusUpdate
from res_plan_server.transport.transport_messages import (
    CommittedLocationsResponseMsg,
    ParticipantDiscoveryMsg,
    PlanProgressMsg,
    RobotOnboardMsg,
)

logger = logging.getLogger("plan_executor")
logging.basicConfig(
    level=logging.DEBUG,
    format="[%(levelname)s] %(asctime)s %(name)s: %(message)s",
)


class PlanExecutor:
    """
    Executes plans.
    """

    def __init__(
        self,
        transport: ExecutorBaseTransport,
        robot_controller: BaseRobotController,
        dependency_manager: DependencyManager,
    ):
        if not isinstance(robot_controller, BaseRobotController):
            raise TypeError

        self._transport = transport
        self._robot_controller = robot_controller
        self._dm = dependency_manager

        self._subscribed_participants: set[str] = set()

        self._message_queue: queue.Queue = queue.Queue()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

        self._transport.subscribe_robot_onboarding(self._on_robot_onboard)

        self._transport.subscribe_participant_discovery(self._on_participant_discovery)

        self._transport.subscribe_committed_locations_request(
            self._on_committed_locations_request
        )

    def _on_robot_onboard(self, message: RobotOnboardMsg) -> None:
        if message.robot_id not in self._subscribed_participants:
            self._subscribed_participants.update(message.robot_id)
            self._subscribe_to_participant(message.robot_id)

    def _on_participant_discovery(self, message: ParticipantDiscoveryMsg) -> None:
        incoming = set(message.participants)
        new_participants = incoming - self._subscribed_participants
        self._subscribed_participants.update(new_participants)
        for participant_id in new_participants:
            self._subscribe_to_participant(participant_id)

    def _subscribe_to_participant(self, participant_id: str) -> None:
        def _on_plan(plan: Plan) -> None:
            logger.info("Received plan")
            self._message_queue.put(("plan", participant_id, plan))

        self._transport.subscribe_plan(
            participant_id,
            _on_plan,
        )
        logger.info("Subscribed to plan topic for %s", participant_id)

    def _on_committed_locations_request(self, request_id: str) -> None:
        self._message_queue.put(("committed_locations_request", request_id))

    def start(self) -> None:
        self._thread.start()
        logger.info("PlanExecutor started")

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join()
        logger.info("PlanExecutor stopped")

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                msg = self._message_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            msg_type = msg[0]

            if msg_type == "plan":
                _, robot_id, plan = msg
                self._handle_plan(robot_id, plan)

            elif msg_type == "waypoint_reached":
                _, robot_id, waypoint_index = msg
                self._handle_waypoint_reached(robot_id, waypoint_index)

            elif msg_type == "committed_locations_request":
                _, request_id = msg
                self._handle_committed_locations_request(request_id)

            elif msg_type == "pause":
                _, robot_ids = msg
                self._handle_pause(robot_ids)

            elif msg_type == "resume":
                _, robot_ids = msg
                self._handle_resume(robot_ids)

            elif msg_type == "error":
                _, robot_id, reason = msg
                self._handle_error(robot_id, reason)

            else:
                logger.warning("Unknown message msg_type: %s", msg_type)

        logger.info("PlanExecutor loop stopped")

    def _make_reached_callback(
        self, robot_id: str, waypoint_index: int
    ) -> Callable[[], None]:
        def _on_reached() -> None:
            self._message_queue.put(("waypoint_reached", robot_id, waypoint_index))

        return _on_reached

    def _dispatch_unblocked(self) -> None:
        for robot_id in self._dm.get_robots():  # TODO Pause/resume
            waypoints = self._dm.get_valid_waypoints_and_advance(robot_id)
            if not waypoints:
                continue

            waypoints_with_callbacks = [
                WaypointWithCallback(
                    location=Location(name=wp.name, x=wp.position[0], y=wp.position[1]),
                    on_reached=self._make_reached_callback(robot_id, i),
                )
                for i, wp in waypoints
            ]
            self._robot_controller.enqueue(robot_id, waypoints_with_callbacks)

            dispatched = [(i, wp.name) for i, wp in waypoints]
            logger.info("Dispatched to %s: %s", robot_id, dispatched)

    def _handle_plan(self, robot_id: str, plan: Plan) -> None:
        logger.info("Handling plan for %s", robot_id)
        existing_plan_state = self._dm.get_plan_state(robot_id)

        if existing_plan_state is None or self._dm.is_complete(robot_id):
            # This is the first plan
            self._dm.set_plan(robot_id, plan)

        elif (
            existing_plan_state.plan.plan_id.destination_session
            == plan.plan_id.destination_session
        ):
            self._dm.update_plan_after_cut(robot_id, plan)
            logger.info(
                "Robot %s plan updated: version %d to %d",
                robot_id,
                existing_plan_state.plan.plan_id.plan_version,
                plan.plan_id.plan_version,
            )

        else:
            logger.info(
                "Robot %s received new session %s while executing session %s",
                robot_id,
                plan.plan_id.destination_session,
                existing_plan_state.plan.plan_id.destination_session,
            )

            # New plan received before previous plan was completed.
            self._dm.update_plan_after_cut(robot_id, plan)

            self._publish_task_status(
                plan.plan_id.destination_session,
                robot_id,
                TaskStatus.SUPERSEDED,
                reason="A new session was received while executing the current one.",
            )

        self._dispatch_unblocked()

    def _handle_waypoint_reached(self, robot_id: str, waypoint_index: int) -> None:
        # TODO: actions
        # TODO: session completion and plan versioning
        logger.debug("%s reached waypoint %d", robot_id, waypoint_index)

        self._dm.update_progress(robot_id, waypoint_index)
        self._publish_progress(robot_id)

        if self._dm.is_complete(robot_id):
            self._on_task_complete(robot_id)

        self._dispatch_unblocked()

    def _handle_committed_locations_request(self, request_id: str) -> None:
        commit_cut = self._dm.compute_commit_cut()

        msg = CommittedLocationsResponseMsg(
            request_id=request_id,
            committed_locations=list(commit_cut.committed_locations.values()),
            stationary_agents=commit_cut.stationary_robots,
        )

        self._transport.publish_committed_locations_response(msg)
        logger.info("Published committed locations for request %s: %s", request_id, msg)

    def _handle_pause(self, robot_ids: set[str]) -> None:
        logger.warning("Pause/resume is not implemented.")

    def _handle_resume(self, robot_ids: set[str]) -> None:
        logger.warning("Pause/resume is not implemented.")

    def _handle_error(self, robot_id: str, reason: str) -> None:
        task_id = self._dm.get_task_id(robot_id)
        self._dm.on_plan_failed(robot_id)
        self._publish_task_status(task_id, robot_id, TaskStatus.FAILED, reason=reason)
        self._transport.publish_plan_error(robot_id, reason)
        logger.error("Robot %s error: %s", robot_id, reason)

    # Task completion
    def _on_task_complete(self, robot_id: str) -> None:
        # Plan / task completion is recorded at the Plan Server
        task_id = self._dm.get_task_id(robot_id)
        self._dm.on_plan_complete(robot_id)
        logger.info("Robot %s completed plan %s", robot_id, task_id)

    # Publishing

    def _publish_progress(self, robot_id: str) -> None:
        state = self._dm.get_plan_state(robot_id)
        if state is None:
            logger.debug("No plan state for %s", robot_id)
            return
        logger.debug("Publishing progress for %s", robot_id)
        self._transport.publish_progress(
            robot_id,
            PlanProgressMsg(
                plan_id=state.plan.plan_id,
                reached_waypoint=state.current_waypoint,
                target_waypoint=state.latest_enqueued,
            ),
        )

    def _publish_robot_task_status(
        self, robot_id: str, status: TaskStatus, reason: Optional[str] = None
    ) -> None:
        task_id = self._dm.get_task_id(robot_id)
        try:
            self._transport.publish_task_status(
                TaskStatusUpdate(
                    task_id=task_id,
                    robot_id=robot_id,
                    status=status,
                    source="plan_executor",
                    reason=reason,
                )
            )
        except Exception:
            logger.exception(
                "Failed to publish task status %s for robot %s", status, robot_id
            )

    def _publish_task_status(
        self,
        task_id: str,
        robot_id: str,
        status: TaskStatus,
        reason: str | None = None,
        superseded_by: str | None = None,
    ) -> None:
        # specify task id
        try:
            self._transport.publish_task_status(
                TaskStatusUpdate(
                    task_id=task_id,
                    robot_id=robot_id,
                    status=status,
                    source="plan_executor",
                    reason=reason,
                    superseded_by=superseded_by,
                )
            )
        except Exception:
            logger.exception("Failed to publish status %s for task %s", status, task_id)
