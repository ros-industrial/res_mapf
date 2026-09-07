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

"""Publish a sample TaskRequest to the example MAPF planner exchange."""

from __future__ import annotations

import argparse
import json
import os

import pika

AMQP_HOST = os.environ.get("AMQP_HOST", "localhost")
AMQP_PORT = int(os.environ.get("AMQP_PORT", "5672"))
AMQP_EXCHANGE = os.environ.get("AMQP_EXCHANGE", "mapf.tasks")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="agent_0")
    parser.add_argument("--task-id", default="t1")
    parser.add_argument("--start", default="0,0")
    parser.add_argument("--goal", default="2,0")
    args = parser.parse_args()

    body = {
        "type": "TaskRequest",
        "robot_id": args.robot_id,
        "task_id": args.task_id,
        "start": args.start,
        "goal": args.goal,
    }

    connection = pika.BlockingConnection(
        pika.ConnectionParameters(host=AMQP_HOST, port=AMQP_PORT)
    )
    channel = connection.channel()
    channel.exchange_declare(
        exchange=AMQP_EXCHANGE, exchange_type="fanout", durable=True
    )
    channel.basic_publish(
        exchange=AMQP_EXCHANGE, routing_key="", body=json.dumps(body).encode()
    )
    connection.close()
    print(f"Published to {AMQP_EXCHANGE}: {body}")


if __name__ == "__main__":
    main()
