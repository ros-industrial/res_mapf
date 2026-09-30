#!/usr/bin/env python3
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

"""AMQP client: publish TaskRequest message(s) to the example MAPF planner.

Default scenario: two agents swap along the same corridor (head-on conflict)::

    agent_0: 0,0 -> 2,0
    agent_1: 2,0 -> 0,0

CBS must resolve vertex/edge conflicts (wait or detour on the 7x7 grid).
"""

from __future__ import annotations

import argparse
import json
import os
from typing import Iterable

import pika

AMQP_HOST = os.environ.get("AMQP_HOST", "localhost")
AMQP_PORT = int(os.environ.get("AMQP_PORT", "5672"))
AMQP_EXCHANGE = os.environ.get("AMQP_EXCHANGE", "mapf.tasks")

# Head-on swap on row y=0 — classic CBS conflict.
DEFAULT_CONFLICT_TASKS = (
    {"robot_id": "agent_0", "task_id": "t1", "start": "0,0", "goal": "2,0"},
    {"robot_id": "agent_1", "task_id": "t2", "start": "2,0", "goal": "0,0"},
)


def _task_body(*, robot_id: str, task_id: str, start: str, goal: str) -> dict[str, str]:
    return {
        "type": "TaskRequest",
        "robot_id": robot_id,
        "task_id": task_id,
        "start": start,
        "goal": goal,
    }


def publish_tasks(tasks: Iterable[dict[str, str]]) -> None:
    connection = pika.BlockingConnection(
        pika.ConnectionParameters(host=AMQP_HOST, port=AMQP_PORT)
    )
    channel = connection.channel()
    channel.exchange_declare(
        exchange=AMQP_EXCHANGE, exchange_type="fanout", durable=True
    )
    for body in tasks:
        channel.basic_publish(
            exchange=AMQP_EXCHANGE, routing_key="", body=json.dumps(body).encode()
        )
        print(f"Published to {AMQP_EXCHANGE}: {body}")
    connection.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--single",
        action="store_true",
        help="Publish one TaskRequest from --robot-id/--task-id/--start/--goal",
    )
    parser.add_argument("--robot-id", default="agent_0")
    parser.add_argument("--task-id", default="t1")
    parser.add_argument("--start", default="0,0")
    parser.add_argument("--goal", default="2,0")
    args = parser.parse_args()

    if args.single:
        tasks = [
            _task_body(
                robot_id=args.robot_id,
                task_id=args.task_id,
                start=args.start,
                goal=args.goal,
            )
        ]
    else:
        tasks = [
            _task_body(
                robot_id=t["robot_id"],
                task_id=t["task_id"],
                start=t["start"],
                goal=t["goal"],
            )
            for t in DEFAULT_CONFLICT_TASKS
        ]

    publish_tasks(tasks)


if __name__ == "__main__":
    main()
