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

"""Example MAPF planner entrypoint: AMQP consumer thread + HTTP server.

AMQP TaskRequest shape (exchange ``mapf.tasks`` / queue ``mapf.planner``)::

    {"type": "TaskRequest", "robot_id": "agent_0", "task_id": "t1",
     "start": "0,0", "goal": "2,0"}

Uses the open 7x7 grid from ``CBSAdapter`` (integer cell ids ``x,y`` with
0 <= x,y <= 6). Serves plans over HTTP ``/preview`` and ``/plans``.
"""

from __future__ import annotations

import logging
import os
import threading

import uvicorn

from amqp_consumer import run_amqp_consumer
from http_server import create_http_app
from plan_store import PlanStore
from planner_service import PlannerService

PLANNER_HTTP_HOST = os.environ.get("PLANNER_HTTP_HOST", "0.0.0.0")
PLANNER_HTTP_PORT = int(os.environ.get("PLANNER_HTTP_PORT", "8090"))
MAP_ID = os.environ.get("MAP_ID", "default_map")
LOGGER = logging.getLogger("planner_app")


def main() -> None:
    logging.basicConfig(
        level=os.environ.get("LOG_LEVEL", "INFO"),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    logging.getLogger("pika").setLevel(logging.WARNING)

    plan_store = PlanStore()
    service = PlannerService(
        map_id=MAP_ID,
        plan_store=plan_store,
    )

    http_app = create_http_app(service, plan_store)

    amqp_thread = threading.Thread(
        target=run_amqp_consumer,
        args=(service,),
        name="amqp-consumer",
        daemon=True,
    )
    amqp_thread.start()

    LOGGER.info(
        "MAPF planner HTTP listening on %s:%s",
        PLANNER_HTTP_HOST,
        PLANNER_HTTP_PORT,
    )
    uvicorn.run(
        http_app,
        host=PLANNER_HTTP_HOST,
        port=PLANNER_HTTP_PORT,
        log_level=os.environ.get("LOG_LEVEL", "info").lower(),
    )


if __name__ == "__main__":
    main()
