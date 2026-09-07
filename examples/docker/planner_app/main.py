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

"""Example MAPF planner: AMQP TaskRequest in, CBS plan on stdout.

Expects JSON messages on exchange ``mapf.tasks`` / queue ``mapf.planner``::

    {"type": "TaskRequest", "robot_id": "agent_0", "task_id": "t1",
     "start": "0,0", "goal": "2,0"}

Uses the open 7x7 grid from ``CBSAdapter`` (integer cell ids ``x,y`` with
0 <= x,y <= 6). Does not publish anything back to AMQP.
"""

from __future__ import annotations

import json
import logging
import os
import time
from typing import Dict
from uuid import uuid4

import pika
from res_mapf_planning.mapf_solve.solvers.cbs_adapter import CBSAdapter
from res_mapf_planning.planning.mapf_coordinator import MAPFCoordinator, PlanningError
from res_mapf_planning.planning.multi_agent_context import MultiAgentContext
from res_mapf_planning.traffic_dependencies.models.plan_id import PlanId
from res_mapf_planning.traffic_dependencies.plan_generator import PlanGenerator
from res_plan_server.models.task import Task
from rich.pretty import pprint

AMQP_HOST = os.environ.get("AMQP_HOST", "localhost")
AMQP_PORT = int(os.environ.get("AMQP_PORT", "5672"))
AMQP_EXCHANGE = os.environ.get("AMQP_EXCHANGE", "mapf.tasks")
AMQP_QUEUE = os.environ.get("AMQP_QUEUE", "mapf.planner")
LOGGER = logging.getLogger("planner_app")


class PlannerService:
    """Consume TaskRequest messages, jointly replan known tasks with CBS."""

    def __init__(self) -> None:
        self.context = MultiAgentContext()
        self.coordinator = MAPFCoordinator(self.context, CBSAdapter())
        self.plan_generator = PlanGenerator()
        self.tasks: Dict[str, Task] = {}

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

        plan_ids = {
            plan.agent_name: PlanId(destination_session=uuid4(), plan_version=1)
            for plan in solver_plans
        }
        plans = self.plan_generator.generate(
            solver_plans=list(solver_plans),
            plan_ids=plan_ids,
            committed_locations=None,
        )
        for plan in plans:
            path = " -> ".join(wp.name for wp in plan.waypoints)
            LOGGER.info("Plan %s: %s", plan.plan_id, path)
            pprint(plan)


def on_message(
    service: PlannerService, _channel, _method, _properties, body: bytes
) -> None:
    try:
        message = json.loads(body)
    except json.JSONDecodeError:
        LOGGER.warning("Non-JSON AMQP body ignored (%d bytes)", len(body))
        return

    if message.get("type") != "TaskRequest":
        LOGGER.debug("Ignoring message type=%s", message.get("type"))
        return

    try:
        service.handle_task_request(message)
    except Exception:
        LOGGER.exception("Failed to handle TaskRequest: %s", body)


def connect_with_retry(
    host: str, port: int, retries: int = 60, delay_s: float = 2.0
) -> pika.BlockingConnection:
    last_error: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            LOGGER.info("Connecting to AMQP %s:%s (attempt %d)", host, port, attempt)
            return pika.BlockingConnection(
                pika.ConnectionParameters(host=host, port=port)
            )
        except (pika.exceptions.AMQPError, OSError) as exc:
            last_error = exc
            LOGGER.warning("AMQP not ready: %s", exc)
            time.sleep(delay_s)
    raise RuntimeError(f"Could not connect to AMQP {host}:{port}") from last_error


def main() -> None:
    logging.basicConfig(
        level=os.environ.get("LOG_LEVEL", "INFO"),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    logging.getLogger("pika").setLevel(logging.WARNING)

    service = PlannerService()
    connection = connect_with_retry(AMQP_HOST, AMQP_PORT)
    channel = connection.channel()
    channel.exchange_declare(
        exchange=AMQP_EXCHANGE, exchange_type="fanout", durable=True
    )
    channel.queue_declare(queue=AMQP_QUEUE, durable=True)
    channel.queue_bind(queue=AMQP_QUEUE, exchange=AMQP_EXCHANGE)
    channel.basic_consume(
        queue=AMQP_QUEUE,
        on_message_callback=lambda ch, method, props, body: on_message(
            service, ch, method, props, body
        ),
        auto_ack=True,
    )

    LOGGER.info(
        "MAPF planner listening on exchange=%s queue=%s (AMQP %s:%s)",
        AMQP_EXCHANGE,
        AMQP_QUEUE,
        AMQP_HOST,
        AMQP_PORT,
    )
    try:
        channel.start_consuming()
    except KeyboardInterrupt:
        LOGGER.info("Shutting down")
    finally:
        if connection.is_open:
            connection.close()


if __name__ == "__main__":
    main()
