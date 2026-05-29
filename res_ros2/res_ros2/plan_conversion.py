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

from datetime import datetime, timezone
from uuid import UUID

from builtin_interfaces.msg import Time as RosTime
from res_mapf_planning.traffic_dependencies.models.plan import Plan
from res_mapf_planning.traffic_dependencies.models.plan_id import PlanId
from res_mapf_planning.traffic_dependencies.models.traffic_dependency import (
    TrafficDependency,
)
from res_mapf_planning.traffic_dependencies.models.waypoint import Waypoint
from rmf_prototype_msgs.msg import Plan as RosPlan
from rmf_prototype_msgs.msg import PlanId as RosPlanId
from rmf_prototype_msgs.msg import TrafficDependency as RosTrafficDependency
from rmf_prototype_msgs.msg import Waypoint as RosWaypoint
from unique_identifier_msgs.msg import UUID as RosUUID


class PlanConversion:
    """Methods to convert to and from ROS 2 Plan-related messages."""

    @staticmethod
    def to_ros_uuid(uuid: UUID) -> RosUUID:
        return RosUUID(uuid=list(uuid.bytes))

    @staticmethod
    def from_ros_uuid(msg: RosUUID) -> UUID:
        return UUID(bytes=bytes(msg.uuid))

    @staticmethod
    def to_ros_plan_id(plan_id: PlanId) -> RosPlanId:
        return RosPlanId(
            destination_session=PlanConversion.to_ros_uuid(plan_id.destination_session),
            plan_version=plan_id.plan_version,
        )

    @staticmethod
    def from_ros_plan_id(msg: RosPlanId) -> PlanId:
        return PlanId(
            destination_session=PlanConversion.from_ros_uuid(msg.destination_session),
            plan_version=msg.plan_version,
        )

    @staticmethod
    def to_ros_dependency(dep: TrafficDependency) -> RosTrafficDependency:
        return RosTrafficDependency(
            name=dep.name,
            plan_id=PlanConversion.to_ros_plan_id(dep.plan_id),
            required_progress=dep.required_progress,
        )

    @staticmethod
    def from_ros_dependency(msg: RosTrafficDependency) -> TrafficDependency:
        return TrafficDependency(
            name=msg.name,
            plan_id=PlanConversion.from_ros_plan_id(msg.plan_id),
            required_progress=msg.required_progress,
        )

    @staticmethod
    def to_ros_waypoint(wp: Waypoint) -> RosWaypoint:
        return RosWaypoint(
            position=list(wp.position),
            progress=wp.progress,
            departure_action=wp.departure_action,
            departure_blockers=[
                PlanConversion.to_ros_dependency(d) for d in wp.departure_blockers
            ],
        )

    @staticmethod
    def from_ros_waypoint(msg: RosWaypoint) -> Waypoint:
        return Waypoint(
            name=f"{msg.position[0]},{msg.position[1]}",
            position=tuple(msg.position),
            progress=msg.progress,
            departure_action=msg.departure_action,
            departure_blockers=[
                PlanConversion.from_ros_dependency(d) for d in msg.departure_blockers
            ],
        )

    @staticmethod
    def to_ros_plan(plan: Plan, to_ros_time) -> RosPlan:
        return RosPlan(
            plan_id=PlanConversion.to_ros_plan_id(plan.plan_id),
            start_time=to_ros_time(plan.start_time),
            workflow=plan.workflow,
            waypoints=[PlanConversion.to_ros_waypoint(w) for w in plan.waypoints],
        )

    @staticmethod
    def from_ros_plan(msg: RosPlan) -> Plan:
        return Plan(
            plan_id=PlanConversion.from_ros_plan_id(msg.plan_id),
            start_time=(msg.start_time),
            workflow=msg.workflow,
            waypoints=[PlanConversion.from_ros_waypoint(w) for w in msg.waypoints],
        )

    @staticmethod
    def from_ros_time(msg: RosTime) -> datetime | None:
        if msg.sec == 0 and msg.nanosec == 0:
            return None
        return datetime.fromtimestamp(msg.sec + msg.nanosec / 1e9, tz=timezone.utc)

    @staticmethod
    def to_ros_time(dt: datetime) -> RosTime:
        if dt is None:
            return RosTime()
        return RosTime(sec=int(dt.timestamp()), nanosec=dt.microsecond * 1000)
