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

from rclpy.node import Node
from res_mapf_planning.traffic_dependencies.models.plan import Plan
from res_plan_server.task_status import TaskStatusUpdate
from res_plan_server.transport.server_base_transport import ServerBaseTransport
from res_plan_server.transport.transport_messages import (
    CommittedLocationMsg,
    CommittedLocationsResponseMsg,
    ParticipantDiscoveryMsg,
    PlanErrorMsg,
    PlanProgressMsg,
    RobotOnboardMsg,
    TaskRequestMsg,
)
from res_ros2_msgs.msg import (
    CommittedLocationsRequest,
    CommittedLocationsResponse,
    RobotOnboard,
    TaskRequest,
)
from res_ros2_msgs.msg import TaskStatus as RosTaskStatus
from res_ros2_msgs.msg import TaskStatusUpdate as RosTaskStatusUpdate
from rmf_prototype_msgs.msg import ParticipantList
from rmf_prototype_msgs.msg import Plan as RosPlan
from rmf_prototype_msgs.msg import PlanError as RosPlanError
from rmf_prototype_msgs.msg import PlanId as RosPlanId
from rmf_prototype_msgs.msg import Progress as RosProgress
from rmf_prototype_msgs.msg import TrafficDependency as RosTrafficDependency
from rmf_prototype_msgs.msg import Waypoint as RosWaypoint
from unique_identifier_msgs.msg import UUID as RosUUID

from .plan_conversion import PlanConversion


class Ros2PlanServerTransport(ServerBaseTransport):
    def __init__(self, node: Node):
        self._node = node
        self._publishers = {}
        self._subscribers = {}

    def _get_or_create_plan_publisher(self, robot_id: str):
        if robot_id not in self._publishers:
            topic = f"/{robot_id}/plan"
            self._publishers[robot_id] = self._node.create_publisher(Plan, topic, 10)
        return self._publishers[robot_id]

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
        self._subscribers["destination_discovery"] = self._node.create_subscription(
            ParticipantList,
            "/destination/discovery",
            lambda msg: callback(
                ParticipantDiscoveryMsg(participants=[p.name for p in msg.participants])
            ),
            10,
        )
        self._node.get_logger().info("Subscribed to participant discovery.")

    def subscribe_task_request(
        self, robot_id: str, callback: Callable[[TaskRequestMsg], None]
    ) -> None:
        topic = f"{robot_id}/task_request"

        if topic in self._subscribers:
            self._node.get_logger().warning(
                f"Already subscribed to task requests for {robot_id} on {topic}"
            )
            return
        self._subscribers[topic] = self._node.create_subscription(
            TaskRequest,
            topic,
            lambda msg: callback(
                TaskRequestMsg(
                    task_id=msg.task_id, robot_id=msg.robot_id, goal=msg.goal
                )
            ),
            10,
        )
        self._node.get_logger().info(
            f"Subscribed to task requests for {robot_id} on {topic}."
        )

    def subscribe_committed_locations_response(
        self, callback: Callable[[CommittedLocationsResponseMsg], None]
    ):
        def _on_message(msg):
            committed_locations = [
                CommittedLocationMsg(
                    robot_id=cl.robot_id,
                    location=cl.location,
                    task_id=cl.task_id,
                    waypoint_index=cl.waypoint_index,
                    progress=cl.progress,
                )
                for cl in msg.committed_locations
            ]

            callback(
                CommittedLocationsResponseMsg(
                    request_id=msg.request_id,
                    committed_locations=committed_locations,
                    stationary_agents=list(msg.stationary_agents),
                )
            )

        self._subscribers["committed_locations_response"] = (
            self._node.create_subscription(
                CommittedLocationsResponse,
                "committed_locations/response",  # TODO: topic name
                _on_message,
                10,
            )
        )
        self._node.get_logger().info("Subscribed to committed locations response.")

    def subscribe_plan_progress(
        self, robot_id: str, callback: Callable[[PlanProgressMsg], None]
    ):
        topic = f"{robot_id}/plan/progress"
        self._subscribers[f"plan_progress/{robot_id}"] = self._node.create_subscription(
            RosProgress,
            topic,
            lambda msg: callback(
                PlanProgressMsg(
                    plan_id=PlanConversion.from_ros_plan_id(msg.plan_id),
                    reached_waypoint=msg.reached_waypoint,
                    target_waypoint=msg.target_waypoint,
                )
            ),
            10,
        )
        self._node.get_logger().info(
            f"Subscribed to plan progress for {robot_id} on {topic}."
        )

    def subscribe_plan_error(
        self, robot_id: str, callback: Callable[[PlanErrorMsg], None]
    ) -> None:
        topic = f"{robot_id}/plan/error"
        self._subscribers[f"plan_error/{robot_id}"] = self._node.create_subscription(
            RosPlanError,
            topic,
            lambda msg: callback(
                PlanErrorMsg(
                    robot_id=robot_id,
                    error_code=msg.code,
                    message=msg.message,
                    details=msg.parameters,
                )
            ),
            10,
        )
        self._node.get_logger().info(
            f"Subscribed to plan errors for {robot_id} on {topic}."
        )

    # Publishers

    def publish_plan(self, robot_id: str, plan: Plan) -> None:
        topic = f"{robot_id}/plan"

        pub = self._get_publisher(topic, RosPlan)

        pub.publish(self._to_ros_plan(plan))

        self._node.get_logger().debug(f"Published plan for {robot_id} on {topic}")

    def publish_committed_locations_request(self, request_id: str) -> None:
        topic = "committed_locations/request"  # TODO: topic name

        pub = self._get_publisher(topic, CommittedLocationsRequest)
        msg = CommittedLocationsRequest(request_id=request_id)
        pub.publish(msg)
        self._node.get_logger().debug(
            f"Published committed locations request {request_id}"
        )

    def publish_task_status(self, update: TaskStatusUpdate) -> None:
        topic = "/fleet/task_status"
        if topic not in self._publishers:
            self._publishers[topic] = self._node.create_publisher(
                RosTaskStatusUpdate, topic, 10
            )

        msg = RosTaskStatusUpdate()
        msg.task_id = update.task_id
        msg.robot_id = update.robot_id
        msg.source = update.source

        task_status = RosTaskStatus()
        task_status.status = update.status.value
        msg.status = task_status
        msg.reason = update.reason if update.reason is not None else ""
        msg.superseded_by = (
            update.superseded_by if update.superseded_by is not None else ""
        )
        msg.timestamp = PlanConversion.to_ros_time(update.timestamp)

        self._publishers[topic].publish(msg)

        self._node.get_logger().debug(
            f"Published task status {update.status.value} for task {update.task_id}"
        )

    def start(self) -> None:
        pass

    def stop(self) -> None:
        for pub in self._publishers.values():
            self._node.destroy_publisher(pub)
        for sub in self._subscribers.values():
            self._node.destroy_subscription(sub)
        self._publishers.clear()
        self._subscribers.clear()

    def _to_ros_plan(self, internal_plan: Plan) -> RosPlan:
        ros_waypoints = []
        for w in internal_plan.waypoints:
            ros_blockers = []
            for d in w.departure_blockers:
                ros_blocker = RosTrafficDependency(
                    name=d.name,
                    plan_id=RosPlanId(
                        destination_session=RosUUID(
                            uuid=list(d.plan_id.destination_session.bytes)
                        ),
                        plan_version=d.plan_id.plan_version,
                    ),
                    required_progress=d.required_progress,
                )
                ros_blockers.append(ros_blocker)

            ros_wp = RosWaypoint(
                position=list(w.position),
                progress=w.progress,
                departure_action=w.departure_action,
                departure_blockers=ros_blockers,
            )
            ros_waypoints.append(ros_wp)

        ros_plan_id = RosPlanId(
            destination_session=RosUUID(
                uuid=list(internal_plan.plan_id.destination_session.bytes)
            ),
            plan_version=internal_plan.plan_id.plan_version,
        )

        return RosPlan(
            plan_id=ros_plan_id,
            start_time=PlanConversion.to_ros_time(internal_plan.start_time),
            workflow=internal_plan.workflow,
            waypoints=ros_waypoints,
        )

    def _get_publisher(self, topic: str, msg_type):
        if topic not in self._publishers:
            self._publishers[topic] = self._node.create_publisher(msg_type, topic, 10)
        return self._publishers[topic]
