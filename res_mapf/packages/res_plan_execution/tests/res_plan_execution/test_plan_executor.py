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

from typing import Callable
from uuid import uuid4

import pytest
from res_mapf_planning.traffic_dependencies.models.plan import Plan, PlanId, Waypoint
from res_plan_execution.plan_execution.dependency_manager import DependencyManager
from res_plan_execution.plan_execution.plan_executor import PlanExecutor
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
    PlanErrorCode,
    PlanErrorMsg,
    PlanIdMsg,
    PlanProgressMsg,
    RobotOnboardMsg,
)


class FakeTransport(ExecutorBaseTransport):
    def __init__(self) -> None:
        self.published_plan_errors: list[tuple[str, PlanErrorMsg]] = []
        self.published_task_statuses: list[TaskStatusUpdate] = []

    def subscribe_robot_onboarding(
        self, callback: Callable[[RobotOnboardMsg], None]
    ) -> None:
        pass

    def subscribe_participant_discovery(
        self, callback: Callable[[ParticipantDiscoveryMsg], None]
    ) -> None:
        pass

    def subscribe_plan(self, robot_id: str, callback: Callable[[Plan], None]) -> None:
        pass

    def subscribe_committed_locations_request(
        self, callback: Callable[[str], None]
    ) -> None:
        pass

    def publish_progress(self, robot_id: str, progress_msg: PlanProgressMsg) -> None:
        pass

    def publish_plan_error(self, robot_id: str, error_msg: PlanErrorMsg) -> None:
        self.published_plan_errors.append((robot_id, error_msg))

    def publish_committed_locations_response(
        self, response_msg: CommittedLocationsResponseMsg
    ) -> None:
        pass

    def publish_task_status(self, status_update: TaskStatusUpdate) -> None:
        self.published_task_statuses.append(status_update)


class FakeRobotController(BaseRobotController):
    def __init__(self) -> None:
        super().__init__(map_data=None)

    def enqueue(
        self, robot_id: str, waypoints_with_callbacks: list[WaypointWithCallback]
    ) -> None:
        pass

    def shutdown(self, interrupted: bool = False) -> None:
        pass


def _drain_one_message(executor: PlanExecutor) -> None:
    msg = executor._message_queue.get_nowait()
    assert msg[0] == "error"
    _, robot_id, error_code, reason = msg
    executor._handle_error(robot_id, error_code, reason)


def _make_plan(plan_id: PlanId, names: tuple[str, str] = ("A", "B")) -> Plan:
    return Plan(
        waypoints=[
            Waypoint(
                name=names[0],
                position=[0, 0],
                progress=0.0,
                departure_blockers=[],
                departure_action="",
            ),
            Waypoint(
                name=names[1],
                position=[1, 0],
                progress=1.0,
                departure_blockers=[],
                departure_action="",
            ),
        ],
        start_time=None,
        plan_id=plan_id,
        workflow="",
    )


def test_handle_error_reports_failed_plans_id_not_a_cleared_one() -> None:
    """
    _handle_error must capture plan_id before calling on_plan_failed
    """
    dm = DependencyManager()
    transport = FakeTransport()
    robot_controller = FakeRobotController()
    executor = PlanExecutor(
        transport=transport, robot_controller=robot_controller, dependency_manager=dm
    )

    plan_id = PlanId(destination_session=uuid4(), plan_version=0)
    dm.set_plan("robot_0", _make_plan(plan_id))

    robot_controller._on_robot_failed(
        "robot_0", PlanErrorCode.PATH_BLOCKED, "obstacle detected on path"
    )
    _drain_one_message(executor)

    assert len(transport.published_plan_errors) == 1
    published_robot_id, error_msg = transport.published_plan_errors[0]
    assert published_robot_id == "robot_0"
    expected_plan_id_msg = PlanIdMsg(
        destination_session=str(plan_id.destination_session),
        plan_version=plan_id.plan_version,
    )
    assert error_msg.plan_id == expected_plan_id_msg, (
        "must report the plan that failed, not None"
    )
    assert error_msg.error_code == PlanErrorCode.PATH_BLOCKED
    assert error_msg.details == "obstacle detected on path"

    assert dm.get_ledger("robot_0").execution_failed, (
        "on_plan_failed must still have run, so nothing more is dispatched "
        "until a replan arrives"
    )


def test_handle_error_publishes_failed_task_status() -> None:
    dm = DependencyManager()
    transport = FakeTransport()
    robot_controller = FakeRobotController()
    executor = PlanExecutor(
        transport=transport, robot_controller=robot_controller, dependency_manager=dm
    )

    plan_id = PlanId(destination_session=uuid4(), plan_version=0)
    dm.set_plan("robot_0", _make_plan(plan_id))

    robot_controller._on_robot_failed(
        "robot_0", PlanErrorCode.INCOMPATIBLE_ACTION, "unsupported action"
    )
    _drain_one_message(executor)

    assert len(transport.published_task_statuses) == 1
    status_update = transport.published_task_statuses[0]
    assert status_update.task_id == str(plan_id.destination_session), (
        "task status is keyed by the destination session, not the plan version"
    )
    assert status_update.robot_id == "robot_0"
    assert status_update.status == TaskStatus.FAILED
    assert status_update.reason == "unsupported action"


def test_handle_error_with_no_plan_state_does_not_publish_plan_error() -> None:
    """
    If the robot has no tracked plan at all, there's no plan_id to report;
    _handle_error must skip publish_plan_error rather than crash.
    """
    dm = DependencyManager()
    transport = FakeTransport()
    robot_controller = FakeRobotController()
    executor = PlanExecutor(
        transport=transport, robot_controller=robot_controller, dependency_manager=dm
    )

    robot_controller._on_robot_failed(
        "unknown_robot", PlanErrorCode.REPLAN_REQUEST, "no plan on file"
    )
    _drain_one_message(executor)

    assert transport.published_plan_errors == []


class RecordingRobotController(FakeRobotController):
    def __init__(self) -> None:
        super().__init__()
        self.enqueued: list[tuple[str, list[WaypointWithCallback]]] = []

    def enqueue(
        self, robot_id: str, waypoints_with_callbacks: list[WaypointWithCallback]
    ) -> None:
        self.enqueued.append((robot_id, waypoints_with_callbacks))


class RejectFirstEnqueueController(RecordingRobotController):
    """Rejects the first enqueue, as a controller would if the robot was not
    ready to start a plan. Records later enqueues."""

    def __init__(self) -> None:
        super().__init__()
        self._has_rejected = False

    def enqueue(
        self, robot_id: str, waypoints_with_callbacks: list[WaypointWithCallback]
    ) -> None:
        if not self._has_rejected:
            self._has_rejected = True
            self._on_robot_failed(
                robot_id,
                PlanErrorCode.ROBOT_NOT_READY,
                "controller rejected the first enqueue",
            )
            return
        super().enqueue(robot_id, waypoints_with_callbacks)


def _drain_one_waypoint_reached(executor: PlanExecutor) -> None:
    msg = executor._message_queue.get_nowait()
    assert msg[0] == "waypoint_reached"
    _, robot_id, reached = msg
    executor._handle_waypoint_reached(robot_id, reached)


def test_robot_can_be_planned_again_after_controller_rejects_first_enqueue() -> None:
    """
    The controller rejects a plan before the robot moves. The robot stays at
    its start, and a new plan from there must be dispatched and completed
    without restarting the executor.
    """
    dm = DependencyManager()
    transport = FakeTransport()
    robot_controller = RejectFirstEnqueueController()
    executor = PlanExecutor(
        transport=transport, robot_controller=robot_controller, dependency_manager=dm
    )

    rejected_plan_id = PlanId(destination_session=uuid4(), plan_version=0)
    executor._handle_plan("robot_0", _make_plan(rejected_plan_id))
    _drain_one_message(executor)

    assert [s.status for s in transport.published_task_statuses] == [
        TaskStatus.FAILED
    ], "the rejected plan must be reported as failed"
    assert len(transport.published_plan_errors) == 1, (
        "the rejected plan must be reported to the plan server"
    )
    assert robot_controller.enqueued == [], (
        "nothing may reach the robot from the rejected plan"
    )

    retry_plan_id = PlanId(destination_session=uuid4(), plan_version=0)
    executor._handle_plan("robot_0", _make_plan(retry_plan_id))

    assert len(robot_controller.enqueued) == 1, (
        "the retry plan must be dispatched after the first plan was rejected"
    )
    robot_id, waypoints = robot_controller.enqueued[0]
    assert robot_id == "robot_0"
    assert [wp.location.name for wp in waypoints] == ["B"], (
        "the robot is still at A, so only B is dispatched"
    )
    assert waypoints[0].task_id == str(retry_plan_id.destination_session), (
        "the dispatched waypoint must belong to the retry plan"
    )

    waypoints[0].on_reached()
    _drain_one_waypoint_reached(executor)

    assert dm.is_complete("robot_0"), "the retry plan must run to completion"
    assert retry_plan_id in dm._completed_plan_ids, (
        "the retry plan must be marked completed so robots blocked on it are released"
    )


@pytest.mark.xfail(
    strict=True,
    reason="_handle_plan publishes SUPERSEDED with the new plan's session, not the old one",
)
def test_new_session_is_not_reported_as_superseded() -> None:
    """
    A plan for a new session arrives while the robot is still executing the
    previous one. The new session is the one taking over, so it must not be
    the one reported as superseded.
    """
    dm = DependencyManager()
    transport = FakeTransport()
    executor = PlanExecutor(
        transport=transport,
        robot_controller=RecordingRobotController(),
        dependency_manager=dm,
    )

    old_plan_id = PlanId(destination_session=uuid4(), plan_version=0)
    executor._handle_plan("robot_0", _make_plan(old_plan_id))

    new_plan_id = PlanId(destination_session=uuid4(), plan_version=0)
    executor._handle_plan("robot_0", _make_plan(new_plan_id, names=("B", "C")))

    superseded_task_ids = [
        s.task_id
        for s in transport.published_task_statuses
        if s.status == TaskStatus.SUPERSEDED
    ]
    assert superseded_task_ids == [str(old_plan_id.destination_session)], (
        "the session that was being executed is the one superseded"
    )


def test_duplicate_reached_callback_from_finished_plan_does_not_advance_next_plan() -> (
    None
):
    """
    The robot finishes a plan A-B and gets a new plan B-C. The controller then
    fires the first plan's callback for B a second time. Both are "waypoint 1",
    so the late callback must be matched by plan id and ignored, not taken as
    the robot having reached C.
    """
    dm = DependencyManager()
    transport = FakeTransport()
    robot_controller = RecordingRobotController()
    executor = PlanExecutor(
        transport=transport, robot_controller=robot_controller, dependency_manager=dm
    )

    first_plan_id = PlanId(destination_session=uuid4(), plan_version=0)
    executor._handle_plan("robot_0", _make_plan(first_plan_id))
    _, first_waypoints = robot_controller.enqueued[0]
    first_waypoints[0].on_reached()
    _drain_one_waypoint_reached(executor)
    assert dm.is_complete("robot_0"), "the first plan must run to completion"

    second_plan_id = PlanId(destination_session=uuid4(), plan_version=0)
    executor._handle_plan("robot_0", _make_plan(second_plan_id, names=("B", "C")))
    assert len(robot_controller.enqueued) == 2, "C must be dispatched"

    first_waypoints[0].on_reached()
    _drain_one_waypoint_reached(executor)

    assert not dm.is_complete("robot_0"), (
        "a late callback from the finished plan must not complete the new plan"
    )
    assert second_plan_id not in dm._completed_plan_ids
