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

"""AMQP transport: consume TaskRequest messages for the MAPF planner."""

from __future__ import annotations

import json
import logging
import os
import time

import pika

from planner_service import PlannerService

AMQP_HOST = os.environ.get("AMQP_HOST", "localhost")
AMQP_PORT = int(os.environ.get("AMQP_PORT", "5672"))
AMQP_EXCHANGE = os.environ.get("AMQP_EXCHANGE", "mapf.tasks")
AMQP_QUEUE = os.environ.get("AMQP_QUEUE", "mapf.planner")
LOGGER = logging.getLogger("planner_app.amqp_consumer")


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


def run_amqp_consumer(service: PlannerService) -> None:
    """Block forever consuming TaskRequest from the planner queue."""
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
    except Exception:
        LOGGER.exception("AMQP consumer stopped")
    finally:
        if connection.is_open:
            connection.close()
