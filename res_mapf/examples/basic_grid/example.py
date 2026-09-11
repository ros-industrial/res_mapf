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

import argparse
import logging
from pathlib import Path
import time
from typing import Callable, Optional

from res_map import lif_parser
from res_map.grid.grid_utils import infer_obstacles, snap_to_grid
from res_mapf_planning.mapf_solve.solvers.cbs_adapter import CBSAdapter
from res_mapf_planning.planning.mapf_coordinator import MAPFCoordinator
from res_mapf_planning.planning.multi_agent_context import MultiAgentContext
from res_mapf_planning.traffic_dependencies.models.plan import Plan
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
    PlanErrorMsg,
    PlanProgressMsg,
    RobotOnboardMsg,
    TaskRequestMsg,
)

logger = logging.getLogger("direct_transport")

"""
Wire publishes and subscribes between Plan Server and Plan Executor directly for testing.

"""


class DirectPlanServerTransport:
    def __init__(self) -> None:
        self._robot_onboarding_callback: Optional[Callable[[RobotOnboardMsg], None]] = (
            None
        )
        self._discovery_callback: Optional[
            Callable[[ParticipantDiscoveryMsg], None]
        ] = None
        self._task_request_callbacks: dict[str, Callable[[TaskRequestMsg], None]] = {}
        self._progress_callbacks: dict[str, Callable[[PlanProgressMsg], None]] = {}

        self._cl_response_callback: Optional[
            Callable[[CommittedLocationsResponseMsg], None]
        ] = None
        self._executor_transport: Optional["DirectPlanExecutorTransport"] = None

    def wire(self, executor_transport: "DirectPlanExecutorTransport") -> None:
        self._executor_transport = executor_transport
        executor_transport._server_transport = self

    # Subscriptions called by Plan Server

    def subscribe_robot_onboarding(
        self, callback: Callable[[RobotOnboardMsg], None]
    ) -> None:
        self._robot_onboarding_callback = callback

    def subscribe_participant_discovery(
        self, callback: Callable[[ParticipantDiscoveryMsg], None]
    ) -> None:
        self._discovery_callback = callback

    def subscribe_task_request(
        self, robot_id: str, callback: Callable[[TaskRequestMsg], None]
    ) -> None:
        self._task_request_callbacks[robot_id] = callback

    def subscribe_committed_locations_response(
        self, callback: Callable[[CommittedLocationsResponseMsg], None]
    ) -> None:
        self._cl_response_callback = callback

    def subscribe_plan_progress(
        self, robot_id: str, callback: Callable[[PlanProgressMsg], None]
    ) -> None:
        self._progress_callbacks[robot_id] = callback

    def subscribe_plan_error(
        self, robot_id: str, callback: Callable[[PlanErrorMsg], None]
    ) -> None:
        pass

    # Publishes called by PlanServer

    def publish_plan(self, robot_id: str, plan: Plan) -> None:
        if self._executor_transport:
            self._executor_transport.deliver_plan(robot_id, plan)

    def publish_committed_locations_request(self, request_id: str) -> None:
        if self._executor_transport:
            self._executor_transport.deliver_cv_request(request_id)

    def publish_task_status(self, update: TaskStatusUpdate) -> None:
        logger.info(
            "TaskStatus: %s %s %s %s",
            update.robot_id,
            update.status.value,
            update.task_id,
            update.reason or "",
        )

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    # Functions to deliver messages to Plan Server

    def deliver_robot_onboard(self, robot_id: str, start: str) -> None:
        # Called by test program
        if self._robot_onboarding_callback:
            logger.info("Calling plan server onboarding")
            self._robot_onboarding_callback(
                RobotOnboardMsg(robot_id=robot_id, start_location=start)
            )

    def deliver_discovery(self, participants: list[str]) -> None:
        # Called by test program
        if self._discovery_callback:
            self._discovery_callback(ParticipantDiscoveryMsg(participants=participants))

    def deliver_task_request(self, robot_id: str, task_id: str, goal: str) -> None:
        # Called by test program
        cb = self._task_request_callbacks.get(robot_id)
        if cb:
            cb(TaskRequestMsg(task_id=task_id, robot_id=robot_id, goal=goal))
        else:
            logger.warning("No task request callback for %s", robot_id)

    def deliver_cv_response(self, response: CommittedLocationsResponseMsg) -> None:
        # Called by DirectPlanExecutorTransport
        if self._cl_response_callback:
            self._cl_response_callback(response)


class DirectPlanExecutorTransport:
    def __init__(self) -> None:
        self._robot_onboarding_callback: Optional[Callable[[RobotOnboardMsg], None]] = (
            None
        )
        self._plan_callbacks: dict[str, Callable[[Plan], None]] = {}
        self._cl_request_callback: Optional[Callable[[str], None]] = None
        self._discovery_callback: Optional[
            Callable[[ParticipantDiscoveryMsg], None]
        ] = None
        self._server_transport: Optional[DirectPlanServerTransport] = None

    # Subscriptions called by Plan Executor

    def subscribe_robot_onboarding(
        self, callback: Callable[[RobotOnboardMsg], None]
    ) -> None:
        self._robot_onboarding_callback = callback

    def subscribe_plan(self, robot_id: str, callback: Callable[[Plan], None]) -> None:
        self._plan_callbacks[robot_id] = callback

    def subscribe_committed_locations_request(
        self, callback: Callable[[str], None]
    ) -> None:
        self._cl_request_callback = callback

    def subscribe_participant_discovery(
        self, callback: Callable[[ParticipantDiscoveryMsg], None]
    ) -> None:
        self._discovery_callback = callback

    # Publishes called by Plan Executor

    def publish_committed_locations_response(
        self, response_msg: CommittedLocationsResponseMsg
    ) -> None:
        if self._server_transport:
            self._server_transport.deliver_cv_response(response_msg)

    def publish_progress(self, robot_id: str, progress_msg: PlanProgressMsg) -> None:
        # deliver progress to PlanServer
        cb = (
            self._server_transport._progress_callbacks.get(robot_id)
            if self._server_transport
            else None
        )
        if cb is not None:
            # logger.info(
            #     "Progress: %s reached=%d target=%d",
            #     robot_id, progress_msg.reached_waypoint, progress_msg.target_waypoint,
            # )
            cb(progress_msg)

    def publish_plan_error(self, robot_id: str, error_msg: PlanErrorMsg) -> None:
        raise NotImplementedError

    def publish_task_status(self, update: TaskStatusUpdate) -> None:
        logger.info("TaskStatusUpdate: %s", update)

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    # Functions to deliver messages to Plan Executor

    def deliver_robot_onboard(self, robot_id: str, start: str) -> None:
        # Called by test program
        if self._robot_onboarding_callback:
            logger.info("Calling plan executor onboarding")
            self._robot_onboarding_callback(
                RobotOnboardMsg(robot_id=robot_id, start_location=start)
            )

    def deliver_plan(self, robot_id: str, plan: Plan) -> None:
        cb = self._plan_callbacks.get(robot_id)
        if cb:
            cb(plan)
        else:
            logger.warning("No plan callback for %s — not yet subscribed", robot_id)

    def deliver_cv_request(self, request_id: str) -> None:
        if self._cl_request_callback:
            self._cl_request_callback(request_id)

    def deliver_discovery(self, participants: list[str]) -> None:
        if self._discovery_callback:
            self._discovery_callback(ParticipantDiscoveryMsg(participants=participants))


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "lif_file",
        type=Path,
        nargs="?",
        default=Path(__file__).parent / "basic_grid.lif.json",
        help="Input .lif.json file. Defaults to the bundled basic_grid map.",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG,
        format="[%(levelname)s] %(asctime)s %(name)s: %(message)s",
    )
    map_data = lif_parser.load_lif(args.lif_file)

    # Convert the map data into a grid map for use by the CBS solver.
    grid_map = snap_to_grid(map_data)
    grid_map.obstacles = infer_obstacles(map_data, grid_map)

    # --- Transports ---
    server_transport = DirectPlanServerTransport()
    executor_transport = DirectPlanExecutorTransport()
    server_transport.wire(executor_transport)

    # --- Plan Server ---
    context = MultiAgentContext()
    solver = CBSAdapter(grid_map)
    coordinator = MAPFCoordinator(context, solver)
    plan_generator = PlanGenerator()

    plan_server = PlanServer(
        transport=server_transport,
        context=context,
        coordinator=coordinator,
        plan_generator=plan_generator,
    )

    # --- Plan Executor ---
    robot_controller = SharedMemoryAgentController(map_data)
    dependency_manager = DependencyManager()

    plan_executor = PlanExecutor(
        transport=executor_transport,
        robot_controller=robot_controller,
        dependency_manager=dependency_manager,
    )

    # --- Register robots ---
    agents = {
        "agent_0": "P_0_0",
        "agent_1": "P_2_0",
    }

    plan_server.start()
    plan_executor.start()

    for robot_id, start in agents.items():
        server_transport.deliver_robot_onboard(robot_id, start)
        executor_transport.deliver_robot_onboard(robot_id, start)
    executor_transport.deliver_discovery(list(agents.keys()))

    time.sleep(0.5)

    # Initial tasks
    server_transport.deliver_task_request("agent_0", "agent_0_task", "P_2_0")
    server_transport.deliver_task_request("agent_1", "agent_1_task", "P_0_0")

    time.sleep(7.0)

    # Replace tasks
    logging.info("Sending replacement tasks...")
    server_transport.deliver_task_request("agent_0", "agent_0_replace", "P_0_3")
    server_transport.deliver_task_request("agent_1", "agent_1_replace", "P_2_2")

    logging.info("Waiting for robots to move..")
    time.sleep(60.0)

    # --- Shutdown ---
    plan_executor.stop()
    plan_server.stop()
    robot_controller.shutdown()


if __name__ == "__main__":
    main()
