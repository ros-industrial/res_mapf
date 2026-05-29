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
import time
from itertools import count
from typing import Dict, Iterable, Iterator, Optional, Tuple
from uuid import uuid4

from res_mapf_planning.mapf_solve.exceptions import MapfSolverError
from res_mapf_planning.planning.mapf_coordinator import MAPFCoordinator
from res_mapf_planning.planning.multi_agent_context import MultiAgentContext
from res_mapf_planning.traffic_dependencies.models.plan import Plan
from res_mapf_planning.traffic_dependencies.models.plan_id import PlanId
from res_mapf_planning.traffic_dependencies.models.progress import Progress
from res_mapf_planning.traffic_dependencies.plan_generator import PlanGenerator
from res_plan_server.models.committed_location import CommittedLocation
from res_plan_server.models.task import Task
from res_plan_server.task_status import TaskStatus, TaskStatusUpdate
from res_plan_server.transport.server_base_transport import ServerBaseTransport
from res_plan_server.transport.transport_messages import (
    CommittedLocationsResponseMsg,
    ParticipantDiscoveryMsg,
    PlanProgressMsg,
    RobotOnboardMsg,
    TaskRequestMsg,
)


class PlanServer:
    """
    Receives task requests, batches them, and calls MAPFCoordinator.
    Publishes plans.
    """

    def __init__(
        self,
        transport: ServerBaseTransport,
        context: MultiAgentContext,
        coordinator: MAPFCoordinator,
        plan_generator: PlanGenerator,
        batch_window: float = 1.0,
        logger: logging.Logger | None = None,
    ):
        self._transport = transport
        self.context = context
        self.coordinator = coordinator
        self.plan_generator = plan_generator
        self._batch_window = batch_window
        self.logger = logger or logging.getLogger(__name__)

        self._request_counter = count(1)
        self._lock = threading.Lock()

        self._subscribed_participants: set[str] = set()

        self._task_id_to_plan_id: Dict[str, PlanId] = {}
        self._robot_to_plan_id: Dict[str, PlanId] = {}  # store current active plan_id

        self._plan_final_waypoints: Dict[PlanId, int] = {}

        self._task_request_queue: queue.Queue = queue.Queue()
        self._committed_locations_response_queue: queue.Queue = queue.Queue()

        self._transport.subscribe_robot_onboarding(self._on_robot_onboard)
        self._transport.subscribe_participant_discovery(self._on_participant_discovery)
        self._transport.subscribe_committed_locations_response(
            self._on_committed_locations_response
        )

    def start(self) -> None:
        self._stop_event = threading.Event()
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        self.logger.info("Plan Server started")

    def stop(self) -> None:
        self._stop_event.set()
        self._worker.join()
        self.logger.info("Plan Server stopped")

    def _on_robot_onboard(self, message: RobotOnboardMsg) -> None:
        self.logger.info(
            "Onboarding robot %s at %s.", message.robot_id, message.start_location
        )

        if message.robot_id not in self._subscribed_participants:
            self._subscribed_participants.update(message.robot_id)
            self._subscribe_to_participant(message.robot_id)
            self._add_robot(message.robot_id, message.start_location)

    def _add_robot(self, robot_id: str, start_location: str) -> None:
        """
        TODO: Must only be called when no robots are moving.
        """
        if self.context.has_executing_agents():
            raise RuntimeError("Cannot add agent while robots are in transit.")
        self.context.initialise_agent(robot_id, start_location)

    # Discovery

    def _on_participant_discovery(self, message: ParticipantDiscoveryMsg) -> None:
        self.logger.info("Participants discovered.")
        incoming = set(message.participants)
        new_participants = incoming - self._subscribed_participants
        self._subscribed_participants.update(new_participants)
        for participant_id in new_participants:
            self._subscribe_to_participant(participant_id)

    def _subscribe_to_participant(self, participant_id: str) -> None:
        self._transport.subscribe_task_request(
            participant_id,
            lambda payload, id=participant_id: self._on_task_request(id, payload),
        )
        self._transport.subscribe_plan_progress(
            participant_id,
            lambda payload, id=participant_id: self._on_progress(id, payload),
        )
        self._transport.subscribe_plan_error(
            participant_id,
            lambda payload, id=participant_id: self._on_plan_error(id, payload),
        )
        self.logger.info(
            "Subscribed to task requests, progress, and errors for %s", participant_id
        )

    # Publish task status

    def _publish_status(
        self,
        task_id: str,
        robot_id: str,
        status: TaskStatus,
        reason: str | None = None,
        superseded_by: str | None = None,
    ) -> None:
        try:
            self._transport.publish_task_status(
                TaskStatusUpdate(
                    task_id=task_id,
                    robot_id=robot_id,
                    status=status,
                    source="plan_server",
                    reason=reason,
                    superseded_by=superseded_by,
                )
            )
        except Exception:
            self.logger.exception(
                "Failed to publish status %s for task %s", status, task_id
            )

    # Process task requests

    def _run(self) -> None:
        while not self._stop_event.is_set():
            batch = self._dequeue(self._task_request_queue, self._batch_window)
            if batch:
                self._process_batch(batch)

        self.logger.info("Plan server run loop exited.")

    def _process_batch(self, batch: list[Task]) -> None:
        # TODO: Manage queuing destinations
        # Add per-task error information.

        self.logger.info(f"Received {len(batch)} tasks")

        # Check for duplicates within this batch.
        seen: dict[str, Task] = {}
        for task in batch:
            if task.robot_id in seen:
                self.logger.info(
                    "Another task received for the same robot within this batch. The first task will be superseded."
                )
                self._publish_status(
                    seen[task.robot_id].task_id,
                    task.robot_id,
                    TaskStatus.SUPERSEDED,
                    reason="Another task request arrived before planning started.",
                    superseded_by=task.task_id,
                )
            seen[task.robot_id] = task

        new_tasks = list(seen.values())
        valid_tasks = []
        for task in new_tasks:
            if not self.context.has_agent(task.robot_id):
                self.logger.warning(
                    "Received task request for uninitialised robot %s, rejecting. Add the robot first.",
                    task.robot_id,
                )
                self._publish_status(
                    task.task_id,
                    task.robot_id,
                    TaskStatus.FAILED,
                    reason=f"Unknown agent {task.robot_id}",
                )
            else:
                valid_tasks.append(task)

        new_tasks = valid_tasks
        self.logger.debug(f"new tasks: {new_tasks}")
        if not new_tasks:
            return

        for task in new_tasks:
            # self.logger.debug("Processing new task: %s", task)
            self._publish_status(
                task_id=task.task_id,
                robot_id=task.robot_id,
                status=TaskStatus.PLANNING,
            )

        # Request committed locations
        request_id = self._next_request_id()
        self._transport.publish_committed_locations_request(request_id)

        response = self._wait_for_committed_locations(request_id, timeout=5.0)

        if response is None:
            for task in new_tasks:
                self._publish_status(
                    task_id=task.task_id,
                    robot_id=task.robot_id,
                    status=TaskStatus.FAILED,
                    reason="Request for committed locations failed.",
                )
            self.logger.error("Request for committed locations failed.")
            return

        committed_locations = self._parse_committed_locations(
            response.committed_locations
        )

        # stationary robots are those known by the Plan Executor to have finished their previous movement task
        # and should remain at their locations.
        # stationary_robots tracks these to set up obstacles in the MAPF scenario.
        # if new tasks have arrived for these robots, remove from the set.
        stationary_robots = set(response.stationary_agents)
        new_task_robot_ids = [task.robot_id for task in new_tasks]

        for new_task_robot in new_task_robot_ids:
            stationary_robots.discard(new_task_robot)

        try:
            plan_ids = {
                task.robot_id: self._get_or_create_plan_id(task.task_id, task.robot_id)
                for task in new_tasks
            }
            # TODO: For now, if the solver is unable to plan for any of the tasks in this batch, the entire batch will be rejected.
            solver_plans = self.coordinator.solve(
                new_tasks=new_tasks,
                committed_locations=committed_locations,
                stationary_agents=stationary_robots,
                obstacles=[],
            )

            self.logger.debug("Solver plans: %s", solver_plans)

            if not solver_plans:
                for task in new_tasks:
                    self._publish_status(
                        task.task_id,
                        task.robot_id,
                        TaskStatus.FAILED,
                        reason="Solver failed to provide a solution.",
                    )
                self.logger.error("Solver was unable to plan for this batch of tasks.")
                return

            generated_plans = self.plan_generator.generate(
                solver_plans, plan_ids, committed_locations
            )

            for robot_id, plan in _plans_by_robot(generated_plans, plan_ids):
                committed_loc = committed_locations.get(robot_id)
                retained = committed_loc.waypoint_index if committed_loc else 0
                self._plan_final_waypoints[plan_ids[robot_id]] = (
                    retained + len(plan.waypoints) - 1
                )
            self.logger.debug(
                "plan final waypoints: %s", str(self._plan_final_waypoints)
            )
            for robot_id, plan in _plans_by_robot(generated_plans, plan_ids):
                self.logger.info("Publishing tasks..")
                self._transport.publish_plan(robot_id, plan)

            # Notify prior tasks that have been replaced by new ones.
            for task in new_tasks:
                existing_agent = self.context.get_agent_snapshot(task.robot_id)
                if (
                    existing_agent
                    and existing_agent.plan_id
                    and existing_agent.plan_id != plan_ids[task.robot_id]
                ):
                    self._publish_status(
                        existing_agent.task_id,
                        task.robot_id,
                        TaskStatus.SUPERSEDED,
                        reason="New task received before previous task was completed.",
                        superseded_by=task.task_id,
                    )
            # Update the agent states
            self.context.on_solve_success(
                committed_locations, new_tasks, plan_ids=plan_ids
            )

            for task in new_tasks:
                self._publish_status(task.task_id, task.robot_id, TaskStatus.PLANNED)

        except MapfSolverError as e:
            self.logger.exception("%e", e)
            for task in new_tasks:
                self._publish_status(
                    task.task_id,
                    task.robot_id,
                    TaskStatus.FAILED,
                    reason="Solver failed to provide a solution.",
                )

        return

    def _wait_for_committed_locations(
        self, request_id: str, timeout: float = 2.0
    ) -> Optional[CommittedLocationsResponseMsg]:
        """
        Block until a committed-locations response matching request_id arrives.
        Returns the message or None if timed out.
        """
        start_time = time.monotonic()
        while True:
            elapsed = time.monotonic() - start_time
            if elapsed > timeout:
                self.logger.error(
                    "Timed out waiting for committed locations for request %s",
                    request_id,
                )
                return None
            try:
                message = self._committed_locations_response_queue.get(
                    timeout=timeout - elapsed
                )
                self.logger.debug("Received committed locations: %s", message)
            except queue.Empty:
                self.logger.error(
                    "Timed out waiting for committed locations for request %s",
                    request_id,
                )
                return None
            if message.request_id == request_id:
                return message

    def _on_task_request(self, robot_id: str, payload: TaskRequestMsg) -> None:
        task = Task(
            task_id=payload.task_id,
            robot_id=payload.robot_id,
            goal=payload.goal,
        )
        self.logger.info("Received task request for %s: %s", robot_id, task)

        try:
            self._task_request_queue.put_nowait(task)
        except queue.Full:
            self._publish_status(
                task_id=task.task_id,
                robot_id=task.robot_id,
                status=TaskStatus.FAILED,
                reason="Plan Server's queue is full.",
                superseded_by=None,
            )
        return

    def _on_committed_locations_response(
        self, message: CommittedLocationsResponseMsg
    ) -> None:
        self._committed_locations_response_queue.put(message)

    def _get_or_create_plan_id(self, task_id: str, robot_id: str) -> PlanId:
        existing = self._task_id_to_plan_id.get(task_id)
        if existing is None:
            plan_id = PlanId(destination_session=uuid4(), plan_version=0)
        else:
            plan_id = PlanId(
                destination_session=existing.destination_session,
                plan_version=existing.plan_version + 1,
            )
        self._task_id_to_plan_id[task_id] = plan_id
        self._robot_to_plan_id[robot_id] = plan_id

        return plan_id

    def _on_progress(self, robot_id: str, payload: PlanProgressMsg) -> None:
        progress = Progress(
            progress=0.0,
            plan_id=payload.plan_id,
            reached_waypoint=payload.reached_waypoint,
            target_waypoint=payload.target_waypoint,
        )

        self.logger.debug("on_progress: Robot %s %s", robot_id, str(progress))
        self.logger.debug("plan final waypoints: %s", str(self._plan_final_waypoints))

        with self._lock:
            final_waypoint = self._plan_final_waypoints.get(progress.plan_id)

            if final_waypoint is None:
                return

            if not (
                progress.reached_waypoint == progress.target_waypoint == final_waypoint
            ):
                return

            task_id = None
            task_id = next(
                (
                    tid
                    for tid, pid in self._task_id_to_plan_id.items()
                    if pid == progress.plan_id
                ),
                None,
            )
            if task_id is not None:
                self._task_id_to_plan_id.pop(task_id)
            self._robot_to_plan_id.pop(
                robot_id, None
            )  # Plan Executor assumes that new UUIDs are given on task completion.
            self._plan_final_waypoints.pop(progress.plan_id, None)

            if task_id is not None:
                self.logger.info(
                    "Robot %s completed task %s", robot_id, progress.plan_id
                )
                self.context.on_completed(robot_id, progress.plan_id)
                self._publish_status(task_id, robot_id, TaskStatus.COMPLETED)

    def _on_plan_error(self, robot_id: str, payload: dict) -> None:
        """ """
        task_id = None
        with self._lock:
            for tid, pid in list(self._task_id_to_plan_id.items()):
                if pid in self._plan_final_waypoints:
                    task_id = tid
                    self._plan_final_waypoints.pop(pid, None)
                    self._task_id_to_plan_id.pop(tid, None)
                    break

        if task_id is not None:
            self.logger.error(
                "Robot %s failed task %s: %s",
                robot_id,
                task_id,
                payload.get("reason"),
            )
            self.context.on_failed(robot_id, task_id)
            self._publish_status(
                task_id=task_id,
                robot_id=robot_id,
                status=TaskStatus.FAILED,
                reason=payload.get("reason"),
            )

    def _next_request_id(self) -> str:
        return f"req_{next(self._request_counter)}"

    @staticmethod
    def _dequeue(
        target_queue: queue.Queue,
        batch_window: float = 1.0,
        timeout: float = 0.5,
        limit: int = 500,
    ) -> list:
        """Blocks for up to 'timeout' seconds if queue is empty.
        When an item is available, takes up to 'batch_window' seconds to pop up to 'limit' items from the queue.
        This function is to be called continually, fetching items from the queue for batch processing.

        Args:
            target_queue (queue.Queue): Message queue
            batch_window (float, optional): Max duration (seconds) to pop items from queue for. Defaults to 1.0.
            timeout (float, optional): Seconds to block waiting for an item to be available. Defaults to 0.5.
            limit (int, optional): Max number of items to pop from queue before returning. Defaults to 500.
        Returns:
            list: List of items from queue.
        """
        items = []
        try:
            items.append(target_queue.get(timeout=timeout))
        except queue.Empty:
            return items

        start_time = time.monotonic()
        while len(items) < limit:
            remaining = batch_window - (time.monotonic() - start_time)
            if remaining <= 0:
                break
            try:
                items.append(target_queue.get(timeout=remaining))
            except queue.Empty:
                break

        return items

    @staticmethod
    def _parse_committed_locations(payload: list) -> Dict[str, CommittedLocation]:
        return {
            item.robot_id: CommittedLocation(
                robot_id=item.robot_id,
                location=item.location,
                task_id=item.task_id,
                progress=item.progress,
                waypoint_index=item.waypoint_index,
            )
            for item in payload
        }


def _plans_by_robot(
    generated_plans: Iterable[Plan], plan_ids: Dict[str, PlanId]
) -> Iterator[Tuple[str, Plan]]:
    plan_id_to_agent = {v: k for k, v in plan_ids.items()}
    for plan in generated_plans:
        if plan.plan_id in plan_id_to_agent:
            yield plan_id_to_agent[plan.plan_id], plan
