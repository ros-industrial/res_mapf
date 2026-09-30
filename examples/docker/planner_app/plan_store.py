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

"""Thread-safe store for latest task and preview plan envelopes."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Literal, Optional

SourceFilter = Literal["all", "task", "preview"]


@dataclass
class StoredEnvelope:
    envelope: dict[str, Any]


class PlanStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest_task: Optional[StoredEnvelope] = None
        self._latest_preview: Optional[StoredEnvelope] = None

    def set_task(self, envelope: dict[str, Any]) -> None:
        with self._lock:
            self._latest_task = StoredEnvelope(envelope=envelope)

    def set_preview(self, envelope: dict[str, Any]) -> None:
        with self._lock:
            self._latest_preview = StoredEnvelope(envelope=envelope)

    def latest_task(self) -> Optional[dict[str, Any]]:
        with self._lock:
            if self._latest_task is None:
                return None
            return self._latest_task.envelope

    def latest_preview(self) -> Optional[dict[str, Any]]:
        with self._lock:
            if self._latest_preview is None:
                return None
            return self._latest_preview.envelope

    def latest_any(self) -> Optional[dict[str, Any]]:
        with self._lock:
            candidates = [
                stored.envelope
                for stored in (self._latest_task, self._latest_preview)
                if stored is not None
            ]
            if not candidates:
                return None
            return max(candidates, key=lambda env: env["updated_at"])

    def get(self, source: SourceFilter) -> Optional[dict[str, Any]]:
        if source == "task":
            return self.latest_task()
        if source == "preview":
            return self.latest_preview()
        return self.latest_any()
