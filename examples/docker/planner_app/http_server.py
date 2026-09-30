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

"""HTTP server: FastAPI routes for MAPF plan preview and stored plans."""

from __future__ import annotations

import logging
from typing import Optional

from fastapi import Body, FastAPI, Query
from pydantic import BaseModel, ConfigDict, Field

from plan_store import PlanStore, SourceFilter
from planner_service import PlannerService

LOGGER = logging.getLogger("planner_app.http_server")

# Same head-on conflict as examples/docker/amqp_publish_task.py
PREVIEW_CONFLICT_EXAMPLE = {
    "tasks": [
        {"robot_id": "agent_0", "start": "0,0", "goal_location": "2,0"},
        {"robot_id": "agent_1", "start": "2,0", "goal_location": "0,0"},
    ]
}


class PreviewTaskSpec(BaseModel):
    robot_id: str = Field(examples=["agent_0"])
    goal_location: str = Field(
        description='Goal cell id "x,y" on the 7x7 grid (0<=x,y<=6).',
        examples=["2,0"],
    )
    start: Optional[str] = Field(
        default=None,
        description='Start cell id "x,y". Optional if the robot already has a known start.',
        examples=["0,0"],
    )


class PreviewRequest(BaseModel):
    model_config = ConfigDict(
        json_schema_extra={"examples": [PREVIEW_CONFLICT_EXAMPLE]},
    )

    tasks: list[PreviewTaskSpec] = Field(
        default_factory=list,
        description="Agents to jointly preview with CBS (same scenario as amqp_publish_task.py).",
    )


class PreviewResponse(BaseModel):
    plans: dict
    error: Optional[str] = None


def create_http_app(service: PlannerService, plan_store: PlanStore) -> FastAPI:
    app = FastAPI(title="MAPF Planner", version="0.1.0")

    @app.get("/health")
    def health() -> dict[str, str]:
        return {"status": "ok"}

    @app.get("/plans")
    def get_plans(
        source: SourceFilter = Query(default="all"),
    ) -> dict:
        envelope = plan_store.get(source)
        if envelope is None:
            return {"type": "plans", "source": source, "plans": {}}
        return envelope

    @app.post("/preview", response_model=PreviewResponse)
    def preview(
        req: PreviewRequest = Body(
            openapi_examples={
                "head_on_conflict": {
                    "summary": "Two agents head-on (same as amqp_publish_task.py)",
                    "description": "agent_0: 0,0→2,0 and agent_1: 2,0→0,0 on the open 7x7 grid.",
                    "value": PREVIEW_CONFLICT_EXAMPLE,
                },
            },
        ),
    ) -> PreviewResponse:
        plans, error = service.preview(
            [task.model_dump() for task in req.tasks],
        )
        return PreviewResponse(plans=plans, error=error)

    return app
