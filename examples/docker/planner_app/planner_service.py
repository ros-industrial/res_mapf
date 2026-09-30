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

"""Shared MAPF planner core: CBS replan (from AMQP) and HTTP preview."""

from __future__ import annotations

import logging
import threading
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional
from uuid import uuid4

from res_mapf_planning.mapf_solve.solvers.cbs_adapter import CBSAdapter
from res_mapf_planning.planning.mapf_coordinator import MAPFCoordinator, PlanningError
from res_mapf_planning.planning.multi_agent_context import MultiAgentContext
from res_mapf_planning.traffic_dependencies.models.plan_id import PlanId
from res_mapf_planning.traffic_dependencies.plan_generator import PlanGenerator
from res_plan_server.models.task import Task
from rich.pretty import pprint

from plan_payload import (
    plan_to_payload,
    plans_to_envelope,
    preview_order_id,
    stationary_preview_payload,
    task_order_id,
)
from plan_store import PlanStore

LOGGER = logging.getLogger("planner_app.planner_service")


class PlannerService:
    """Joint CBS replan for known tasks; used by AMQP consumer and HTTP server."""

    def __init__(
        self,
        *,
        map_id: str,
        plan_store: PlanStore,
    ) -> None:
        self.map_id = map_id
        self.plan_store = plan_store
        self.context = MultiAgentContext()
        self.coordinator = MAPFCoordinator(self.context, CBSAdapter())
        self.plan_generator = PlanGenerator()
        self.tasks: Dict[str, Task] = {}
        self._task_plan_version = 0
        self._preview_lock = threading.Lock()

    def _store(self, envelope: dict, *, source: str) -> None:
        if source == "task":
            self.plan_store.set_task(envelope)
        elif source == "preview":
            self.plan_store.set_preview(envelope)

    def handle_task_request(self, message: dict) -> None:
        robot_id = str(message.get("robot_id", ""))
        task_id = str(message.get("task_id", "") or f"{robot_id}_task")
        start = str(message.get("start", ""))
        goal = str(message.get("goal", ""))
        if not robot_id or not start or not goal:
            LOGGER.warning(
                "Ignoring TaskRequest missing robot_id/start/goal: %s", message
            )
            return

        self.context.initialise_agent(robot_id, start)
        self.tasks[robot_id] = Task(task_id=task_id, robot_id=robot_id, goal=goal)
        LOGGER.info(
            "TaskRequest %s: %s %s -> %s (%d known task(s))",
            task_id,
            robot_id,
            start,
            goal,
            len(self.tasks),
        )
        self.replan()

    def replan(self) -> None:
        tasks = list(self.tasks.values())
        if not tasks:
            return
        try:
            solver_plans = self.coordinator.solve(
                new_tasks=tasks,
                committed_locations={},
                stationary_agents=set(),
                obstacles=[],
            )
        except (PlanningError, Exception):
            LOGGER.exception("CBS planning failed")
            return

        if not solver_plans:
            LOGGER.error("Coordinator returned no plans")
            return

        self._task_plan_version += 1
        plan_version = self._task_plan_version
        plan_ids = {
            plan.agent_name: PlanId(
                destination_session=uuid4(), plan_version=plan_version
            )
            for plan in solver_plans
        }
        self.plan_generator.generate(
            solver_plans=list(solver_plans),
            plan_ids=plan_ids,
            committed_locations=None,
        )
        task_by_robot = {task.robot_id: task for task in tasks}
        payloads: Dict[str, dict[str, Any]] = {}
        for robot_id, plan in self.plan_generator.plans_dict.items():
            task = task_by_robot.get(robot_id)
            task_id = task.task_id if task else ""
            payload = plan_to_payload(
                robot_id,
                plan,
                order_id=task_order_id(robot_id, plan_version),
                map_id=self.map_id,
                plan_version=plan_version,
                task_id=task_id,
                preview=False,
            )
            payloads[robot_id] = payload
            path = " -> ".join(wp["name"] for wp in payload["waypoints"])
            LOGGER.info("Plan %s: %s", robot_id, path)
            pprint(plan)

        updated_at = datetime.now(timezone.utc)
        envelope = plans_to_envelope(
            payloads,
            source="task",
            map_id=self.map_id,
            updated_at=updated_at,
        )
        self._store(envelope, source="task")

    def preview(
        self, task_specs: List[dict]
    ) -> tuple[Dict[str, dict[str, Any]], Optional[str]]:
        """Dry-run CBS for UI preview — does not dispatch or mutate task batch."""
        with self._preview_lock:
            warnings: list[str] = []
            context = MultiAgentContext()
            preview_coordinator = MAPFCoordinator(context, CBSAdapter())
            preview_generator = PlanGenerator()
            task_list: List[Task] = []
            preview_starts: Dict[str, str] = {}

            for spec in task_specs:
                robot_id = str(spec.get("robot_id", ""))
                goal = str(spec.get("goal_location", ""))
                start = spec.get("start")
                if not robot_id or not goal:
                    warnings.append(
                        f"Skipping task missing robot_id/goal_location: {spec}"
                    )
                    continue

                if start:
                    start_str = str(start)
                else:
                    snapshot = self.context.get_agent_snapshot(robot_id)
                    start_str = (
                        snapshot.start_location
                        if snapshot and snapshot.start_location
                        else ""
                    )
                if not start_str:
                    warnings.append(f"No start for {robot_id}")
                    continue

                context.initialise_agent(robot_id, start_str)
                preview_starts[robot_id] = start_str
                task_list.append(
                    Task(
                        task_id=f"preview_{robot_id}",
                        robot_id=robot_id,
                        goal=goal,
                    )
                )

            if not task_list:
                error = "; ".join(warnings) if warnings else "No valid preview tasks"
                return {}, error

            try:
                solver_plans = preview_coordinator.solve(
                    new_tasks=task_list,
                    committed_locations={},
                    stationary_agents=set(),
                    obstacles=[],
                )
            except (PlanningError, Exception) as exc:
                LOGGER.exception("preview CBS failed")
                return {}, str(exc)

            if not solver_plans:
                return {}, "preview CBS returned no plans"

            plan_ids = {
                plan.agent_name: PlanId(destination_session=uuid4(), plan_version=0)
                for plan in solver_plans
            }
            preview_generator.generate(
                solver_plans=list(solver_plans),
                plan_ids=plan_ids,
                committed_locations=None,
            )

            result: Dict[str, dict[str, Any]] = {}
            for robot_id, plan in preview_generator.plans_dict.items():
                payload = plan_to_payload(
                    robot_id,
                    plan,
                    order_id=preview_order_id(robot_id),
                    map_id=self.map_id,
                    plan_version=0,
                    task_id="",
                    preview=True,
                )
                result[robot_id] = payload
                path = " -> ".join(wp["name"] for wp in payload["waypoints"])
                LOGGER.info("preview plan %s: %s", robot_id, path)

            for robot_id, start in preview_starts.items():
                if robot_id in result:
                    continue
                payload = stationary_preview_payload(
                    robot_id, start, map_id=self.map_id
                )
                result[robot_id] = payload
                LOGGER.info("preview idle %s: %s", robot_id, start)

            updated_at = datetime.now(timezone.utc)
            envelope = plans_to_envelope(
                result,
                source="preview",
                map_id=self.map_id,
                updated_at=updated_at,
            )
            self._store(envelope, source="preview")
            error = "; ".join(warnings) if warnings else None
            return result, error
