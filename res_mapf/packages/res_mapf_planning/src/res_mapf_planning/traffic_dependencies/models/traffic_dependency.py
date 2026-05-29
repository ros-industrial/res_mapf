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

from dataclasses import dataclass

from .plan_id import PlanId


@dataclass(frozen=True)
class TrafficDependency:
    name: str  # name of other agent to wait for
    plan_id: PlanId
    required_progress: float

    def is_satisfied(self, other_progress: float) -> bool:
        """
        Check if the dependency has been satisfied.

        Args:
            other_progress: Progress of the blocking plan

        Returns:
            True if the blocker is cleared
        """
        return other_progress >= self.required_progress
