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

from abc import ABC, abstractmethod
from typing import Callable

from res_mapf_planning.traffic_dependencies.models.plan import Plan
from res_plan_server.task_status import TaskStatusUpdate
from res_plan_server.transport.transport_messages import (
    CommittedLocationsResponseMsg,
    ParticipantDiscoveryMsg,
    PlanErrorMsg,
    PlanProgressMsg,
    RobotOnboardMsg,
    TaskRequestMsg,
)


class ServerBaseTransport(ABC):
    """Base class for pub/sub transport."""

    @abstractmethod
    def subscribe_robot_onboarding(
        self, callback: Callable[[RobotOnboardMsg], None]
    ) -> None:
        """
        Subscribe to the robot onboarding topic.
        """
        ...

    @abstractmethod
    def subscribe_participant_discovery(
        self, callback: Callable[[ParticipantDiscoveryMsg], None]
    ) -> None:
        """
        Subscribe to the participant discovery topic.
        """
        ...

    @abstractmethod
    def subscribe_task_request(
        self, robot_id: str, callback: Callable[[TaskRequestMsg], None]
    ) -> None:
        """
        Subscribe to task requests for a single robot.
        """
        ...

    @abstractmethod
    def subscribe_committed_locations_response(
        self, callback: Callable[[CommittedLocationsResponseMsg], None]
    ) -> None:
        """
        Subscribe to response for committed locations.
        """
        ...

    @abstractmethod
    def subscribe_plan_progress(
        self, robot_id: str, callback: Callable[[PlanProgressMsg], None]
    ) -> None:
        """
        Subscribe to execution progress for a single robot.
        """

    @abstractmethod
    def subscribe_plan_error(
        self, robot_id: str, callback: Callable[[PlanErrorMsg], None]
    ) -> None:
        """
        Subscribe to plan execution errors for a single robot.
        """

    @abstractmethod
    def publish_plan(self, robot_id: str, plan: Plan) -> None:
        """Publish plan message to an robot-specific channel."""
        ...

    @abstractmethod
    def publish_committed_locations_request(self, request_id: str) -> None: ...

    @abstractmethod
    def publish_task_status(self, update: TaskStatusUpdate) -> None: ...

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...
