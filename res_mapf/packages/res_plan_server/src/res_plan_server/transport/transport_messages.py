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

from dataclasses import dataclass, field


@dataclass(frozen=True)
class RobotOnboardMsg:
    robot_id: str
    start_location: str


@dataclass(frozen=True)
class ParticipantDiscoveryMsg:
    participants: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class TaskRequestMsg:
    task_id: str
    robot_id: str
    goal: str


@dataclass(frozen=True)
class PlanIdMsg:
    destination_uuid: str
    plan_version: int


@dataclass(frozen=True)
class CommittedLocationMsg:
    robot_id: str
    location: str
    waypoint_index: int
    progress: float
    task_id: str


@dataclass(frozen=True)
class CommittedLocationsResponseMsg:
    request_id: str
    committed_locations: list[CommittedLocationMsg] = field(default_factory=list)
    stationary_agents: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class PlanProgressMsg:
    plan_id: PlanIdMsg
    reached_waypoint: int
    target_waypoint: int


@dataclass(frozen=True)
class PlanErrorMsg:
    plan_id: PlanIdMsg
    error_code: int
    details: str
