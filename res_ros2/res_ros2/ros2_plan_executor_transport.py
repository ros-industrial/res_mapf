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

import json

from rclpy.node import Callable, Node
from res_mapf_planning.traffic_dependencies.models.plan import Plan
from res_plan_execution.plan_execution.transport.executor_base_transport import (
    ExecutorBaseTransport,
)
from res_plan_server.transport.transport_messages import (
    CommittedLocationsResponseMsg,
    ParticipantDiscoveryMsg,
    PlanErrorMsg,
    PlanProgressMsg,
    RobotOnboardMsg,
)
from res_ros2.plan_conversion import PlanConversion
from res_ros2_msgs.msg import (
    CommittedLocationsRequest,
    CommittedLocationsResponse,
    CommittedLocation,
    RobotOnboard,
)
from rmf_prototype_msgs.msg import ParticipantList
from rmf_prototype_msgs.msg import (
    Plan as RosPlan,
    PlanId as RosPlanId,
    PlanError as RosPlanError,
    Error as RosError,
)
from rmf_prototype_msgs.msg import Progress as RosProgress
from unique_identifier_msgs.msg import UUID as RosUUID


class Ros2PlanExecutorTransport(ExecutorBaseTransport):
    def __init__(self, node: Node):
        self._node = node
        self._publishers = {}
        self._subscribers = {}

    def subscribe_robot_onboarding(
        self, callback: Callable[[RobotOnboardMsg], None]
    ) -> None:
        self._subscribers["robot_onboarding"] = self._node.create_subscription(
            RobotOnboard,
            "/robot_onboard",
            lambda msg: callback(
                RobotOnboardMsg(
                    robot_id=msg.robot_id, start_location=msg.start_location
                )
            ),
            10,
        )
        self._node.get_logger().info("Subscribed to robot_onboard.")

    def subscribe_participant_discovery(
        self, callback: Callable[[ParticipantDiscoveryMsg], None]
    ) -> None:
        """
        Subscribe to the participant discovery topic.
        """
        topic = "/destination/discovery"
        self._subscribers["destination_discovery"] = self._node.create_subscription(
            ParticipantList,
            topic,
            lambda msg: callback(
                ParticipantDiscoveryMsg(participants=[p.name for p in msg.participants])
            ),
            10,
        )
        self._node.get_logger().info("Subscribed to participant discovery")

    def subscribe_plan(self, robot_id: str, callback: Callable[[Plan], None]) -> None:
        topic = f"/{robot_id}/plan"
        self._subscribers[f"plan/{robot_id}"] = self._node.create_subscription(
            RosPlan,
            topic,
            lambda msg: callback(PlanConversion.from_ros_plan(msg)),
            10,
        )

    def subscribe_committed_locations_request(
        self, callback: Callable[[str], None]
    ) -> None:
        topic = "/committed_locations/request"
        self._node.create_subscription(
            CommittedLocationsRequest,
            topic,
            lambda msg: callback(msg.request_id),
            10,
        )

    def publish_progress(self, robot_id: str, progress_msg: PlanProgressMsg) -> None:
        topic = f"/{robot_id}/plan/progress"
        pub = self._get_publisher(topic, RosProgress)

        ros_msg = RosProgress()
        ros_msg.plan_id = RosPlanId(
            destination_session=RosUUID(
                uuid=list(progress_msg.plan_id.destination_session.bytes)
            ),
            plan_version=progress_msg.plan_id.plan_version,
        )
        progress_msg.plan_id
        ros_msg.reached_waypoint = progress_msg.reached_waypoint
        ros_msg.target_waypoint = progress_msg.target_waypoint

        pub.publish(ros_msg)

    def publish_plan_error(self, robot_id: str, error_msg: PlanErrorMsg) -> None:
        topic = f"/{robot_id}/plan/error"
        pub = self._get_publisher(topic, RosPlanError)

        ros_msg = RosPlanError()
        ros_msg.error = RosError(
            code=error_msg.error_code, message=error_msg.details, parameters=""
        )
        ros_msg.plan_id = error_msg.plan_id
        ros_msg.data = json.dumps(error_msg.details)

        pub.publish(ros_msg)

    def publish_committed_locations_response(
        self, response_msg: CommittedLocationsResponseMsg
    ) -> None:
        topic = "/committed_locations/response"
        pub = self._get_publisher(topic, CommittedLocationsResponse)

        ros_msg = CommittedLocationsResponse()
        ros_msg.request_id = response_msg.request_id
        ros_msg.committed_locations = [
            CommittedLocation(
                robot_id=c.robot_id,
                location=c.location,
                task_id=c.task_id,
                waypoint_index=c.waypoint_index,
            )
            for c in response_msg.committed_locations
        ]
        ros_msg.stationary_agents = response_msg.stationary_agents

        pub.publish(ros_msg)

    def _get_publisher(self, topic: str, msg_type):
        if topic not in self._publishers:
            self._publishers[topic] = self._node.create_publisher(msg_type, topic, 10)
        return self._publishers[topic]
