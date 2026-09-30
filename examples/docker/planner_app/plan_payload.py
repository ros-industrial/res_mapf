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

"""Serialize Plan objects to blue-ocean-compatible MAPF plan payloads."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Dict, Literal, Optional
from uuid import uuid4

from res_mapf_planning.traffic_dependencies.models.plan import Plan

PlanSource = Literal["task", "preview"]


def _node_xy(node_id: str) -> tuple[float, float]:
    parts = node_id.split(",")
    return float(parts[0]), float(parts[1])


def plan_to_payload(
    robot_id: str,
    plan: Plan,
    *,
    order_id: str,
    map_id: str,
    plan_version: int,
    task_id: str,
    preview: bool = False,
) -> dict[str, Any]:
    """Plan -> per-robot payload (MapfPlan shape for the UI)."""
    waypoints: list[dict[str, Any]] = []
    for waypoint in plan.waypoints:
        blockers = [
            {
                "robot_id": blocker.name,
                "required_progress": blocker.required_progress,
                "plan_version": blocker.plan_id.plan_version,
            }
            for blocker in waypoint.departure_blockers
        ]
        if waypoints and waypoints[-1]["name"] == waypoint.name:
            waypoints[-1]["departure_blockers"].extend(blockers)
            waypoints[-1]["progress"] = waypoint.progress
            continue
        waypoints.append(
            {
                "name": waypoint.name,
                "x": waypoint.position[0],
                "y": waypoint.position[1],
                "progress": waypoint.progress,
                "departure_blockers": blockers,
            }
        )

    payload: dict[str, Any] = {
        "robot_id": robot_id,
        "order_id": order_id,
        "map_id": map_id,
        "plan_version": plan_version,
        "task_id": task_id,
        "waypoints": waypoints,
    }
    if preview:
        payload["preview"] = True
    return payload


def stationary_preview_payload(
    robot_id: str,
    node_id: str,
    *,
    map_id: str,
) -> dict[str, Any]:
    """Idle robot at a single node (no CBS path)."""
    x, y = _node_xy(node_id)
    return {
        "robot_id": robot_id,
        "order_id": f"preview_{robot_id}",
        "map_id": map_id,
        "plan_version": 0,
        "task_id": "",
        "preview": True,
        "waypoints": [
            {
                "name": node_id,
                "x": x,
                "y": y,
                "progress": 0.0,
                "departure_blockers": [],
            }
        ],
    }


def plans_to_envelope(
    plans: Dict[str, dict[str, Any]],
    *,
    source: PlanSource,
    map_id: str,
    updated_at: Optional[datetime] = None,
) -> dict[str, Any]:
    """Wrap per-robot payloads in a WS/HTTP envelope."""
    ts = updated_at or datetime.now(timezone.utc)
    return {
        "type": "plans",
        "source": source,
        "map_id": map_id,
        "updated_at": ts.isoformat(),
        "plans": plans,
    }


def task_order_id(robot_id: str, plan_version: int) -> str:
    return f"mapf_{robot_id}_v{plan_version}_{uuid4().hex[:8]}"


def preview_order_id(robot_id: str) -> str:
    return f"preview_{robot_id}"
