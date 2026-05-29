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
from datetime import datetime, timezone
from enum import Enum


class TaskStatus(Enum):
    # determined by Plan Server
    PLANNING = 1  # request accepted, solver running
    PLANNED = 2  # plan published
    SUPERSEDED = 3  # superseded by a newer request before planning finished

    # determined by Plan Executor
    IN_PROGRESS = 4
    COMPLETED = 5
    AWAITING_REPLAN = 6
    PAUSED = 7
    EXECUTOR_PAUSED = 8

    FAILED = 9


@dataclass
class TaskStatusUpdate:
    task_id: str
    robot_id: str
    status: TaskStatus
    source: str  # "plan_server" | "plan_executor"
    reason: str | None = None
    superseded_by: str | None = (
        None  # task id of superseding task. only set when status=SUPERSEDED
    )
    timestamp: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
