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
import time

from res_mapf_planning.mapf_solve.solvers.cbs_adapter import CBSAdapter
from res_mapf_planning.planning.mapf_coordinator import MAPFCoordinator
from res_mapf_planning.planning.multi_agent_context import MultiAgentContext
from res_mapf_planning.traffic_dependencies.plan_generator import PlanGenerator
from res_plan_execution.plan_execution.dependency_manager import DependencyManager
from res_plan_execution.plan_execution.plan_executor import PlanExecutor
from res_plan_execution.robot_controllers.shared_memory_agent_controller.agent_shmd_controller import (
    SharedMemoryAgentController,
)
from res_plan_server.plan_server import PlanServer
from res_plan_server.task_status import TaskStatusUpdate
from res_plan_server.transport.transport_messages import (
    CommittedLocationsResponseMsg,
    ParticipantDiscoveryMsg,
    PlanProgressMsg,
    RobotOnboardMsg,
    TaskRequestMsg,
)

logger = logging.getLogger("direct_transport")

"""
Wire publishes and subscribes between Plan Server and Plan Executor directly for testing.

python3 src/res_ros2_ws/res_ros2/res_ros2/test/integration.py
"""


class DirectPlanServerTransport:
    def __init__(self):
        self._discovery_callback = None
        self._task_request_callbacks = {}
        self._progress_callbacks = {}

        self._cl_response_callback = None
        self._executor_transport = None
        self._progress_callback = None

    def wire(self, executor_transport: "DirectPlanExecutorTransport"):
        self._executor_transport = executor_transport
        executor_transport._server_transport = self

    # Subscriptions called by Plan Server

    def subscribe_robot_onboarding(self, callback):
        self._robot_onboarding_callback = callback

    def subscribe_participant_discovery(self, callback):
        self._discovery_callback = callback

    def subscribe_task_request(self, robot_id, callback):
        self._task_request_callbacks[robot_id] = callback

    def subscribe_committed_locations_response(self, callback):
        self._cl_response_callback = callback

    def subscribe_plan_progress(self, robot_id, callback):
        self._progress_callbacks[robot_id] = callback

    def subscribe_plan_error(self, robot_id, callback):
        pass

    # Publishes called by PlanServer

    def publish_plan(self, robot_id, plan):
        if self._executor_transport:
            self._executor_transport.deliver_plan(robot_id, plan)

    def publish_committed_locations_request(self, request_id):
        if self._executor_transport:
            self._executor_transport.deliver_cv_request(request_id)

    def publish_task_status(self, update: TaskStatusUpdate):
        logger.info(
            "TaskStatus: %s %s %s %s",
            update.robot_id,
            update.status.value,
            update.task_id,
            update.reason or "",
        )

    def start(self):
        pass

    def stop(self):
        pass

    # Functions to deliver messages to Plan Server

    def deliver_robot_onboard(self, robot_id, start):
        # Called by test program
        if self._robot_onboarding_callback:
            logger.info("Calling plan server onboarding")
            self._robot_onboarding_callback(
                RobotOnboardMsg(robot_id=robot_id, start_location=start)
            )

    def deliver_discovery(self, participants: list[str]):
        # Called by test program
        if self._discovery_callback:
            self._discovery_callback(ParticipantDiscoveryMsg(participants=participants))

    def deliver_task_request(self, robot_id: str, task_id: str, goal: str):
        # Called by test program
        cb = self._task_request_callbacks.get(robot_id)
        if cb:
            cb(TaskRequestMsg(task_id=task_id, robot_id=robot_id, goal=goal))
        else:
            logger.warning("No task request callback for %s", robot_id)

    def deliver_cv_response(self, response: CommittedLocationsResponseMsg):
        # Called by DirectPlanExecutorTransport
        if self._cl_response_callback:
            self._cl_response_callback(response)


class DirectPlanExecutorTransport:
    def __init__(self):
        self._plan_callbacks = {}  # robot_id: callback
        self._cl_request_callback = None
        self._discovery_callback = None
        self._server_transport = None

    # Subscriptions called by Plan Executor

    def subscribe_robot_onboarding(self, callback):
        self._robot_onboarding_callback = callback

    def subscribe_plan(self, robot_id, callback):
        self._plan_callbacks[robot_id] = callback

    def subscribe_committed_locations_request(self, callback):
        self._cl_request_callback = callback

    def subscribe_participant_discovery(self, callback):
        self._discovery_callback = callback

    # Publishes called by Plan Executor

    def publish_committed_locations_response(self, response_msg):
        if self._server_transport:
            self._server_transport.deliver_cv_response(response_msg)

    def publish_progress(self, robot_id: str, progress_msg: PlanProgressMsg):
        # deliver progress to PlanServer
        if (
            self._server_transport
            and self._server_transport._progress_callbacks[robot_id]
        ):
            # logger.info(
            #     "Progress: %s reached=%d target=%d",
            #     robot_id, progress_msg.reached_waypoint, progress_msg.target_waypoint,
            # )
            self._server_transport._progress_callbacks[robot_id](progress_msg)

    def publish_plan_error(self, robot_id, reason):
        raise NotImplementedError

    def publish_task_status(self, update: TaskStatusUpdate):
        logger.info("TaskStatusUpdate: %s", update)

    def start(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    # Functions to deliver messages to Plan Executor

    def deliver_robot_onboard(self, robot_id, start):
        # Called by test program
        if self._robot_onboarding_callback:
            logger.info("Calling plan executor onboarding")
            self._robot_onboarding_callback(
                RobotOnboardMsg(robot_id=robot_id, start_location=start)
            )

    def deliver_plan(self, robot_id: str, plan):
        cb = self._plan_callbacks.get(robot_id)
        if cb:
            cb(plan)
        else:
            logger.warning("No plan callback for %s — not yet subscribed", robot_id)

    def deliver_cv_request(self, request_id: str):
        if self._cl_request_callback:
            self._cl_request_callback(request_id)

    def deliver_discovery(self, participants: list[str]):
        if self._discovery_callback:
            self._discovery_callback(ParticipantDiscoveryMsg(participants=participants))


def main():
    logging.basicConfig(
        level=logging.DEBUG,
        format="[%(levelname)s] %(asctime)s %(name)s: %(message)s",
    )

    # --- Transports ---
    server_transport = DirectPlanServerTransport()
    executor_transport = DirectPlanExecutorTransport()
    server_transport.wire(executor_transport)

    # --- Plan Server ---
    context = MultiAgentContext()
    solver = CBSAdapter()
    coordinator = MAPFCoordinator(context, solver)
    plan_generator = PlanGenerator()

    plan_server = PlanServer(
        transport=server_transport,
        context=context,
        coordinator=coordinator,
        plan_generator=plan_generator,
    )

    # --- Plan Executor ---
    robot_controller = SharedMemoryAgentController()
    dependency_manager = DependencyManager()

    plan_executor = PlanExecutor(
        transport=executor_transport,
        robot_controller=robot_controller,
        dependency_manager=dependency_manager,
    )

    # --- Register robots ---
    agents = {
        "agent_0": "0,0",
        "agent_1": "2,0",
    }

    plan_server.start()
    plan_executor.start()

    for robot_id, start in agents.items():
        server_transport.deliver_robot_onboard(robot_id, start)
        executor_transport.deliver_robot_onboard(robot_id, start)
    executor_transport.deliver_discovery(list(agents.keys()))

    time.sleep(0.5)

    # Initial tasks
    server_transport.deliver_task_request("agent_0", "agent_0_task", "2,0")
    server_transport.deliver_task_request("agent_1", "agent_1_task", "0,0")

    time.sleep(1.5)

    # Replace tasks
    logging.info("Sending replacement tasks...")
    server_transport.deliver_task_request("agent_0", "agent_0_replace", "1,2")
    server_transport.deliver_task_request("agent_1", "agent_1_replace", "0,3")

    logging.info("Waiting for robots to move..")
    time.sleep(60.0)

    # --- Shutdown ---
    plan_executor.stop()
    plan_server.stop()
    robot_controller.shutdown()


if __name__ == "__main__":
    main()
