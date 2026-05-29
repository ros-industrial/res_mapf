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

import logging
import os
import uuid
from typing import Any, Dict, List, TypedDict

import requests
import yaml

from res_mapf_planning.mapf_solve.mapf_solver_base import (
    MAPFAgent,
    MAPFSolverBase,
    Obstacle,
)
from res_mapf_planning.mapf_solve.models.models import SolverPlan

logger = logging.getLogger("MAPFServiceAdapter")


class _Position(TypedDict):
    x: int
    y: int


class _Task(TypedDict):
    agent_name: str
    start_position: _Position
    end_position: _Position


class _ObstacleEntry(TypedDict):
    coordinate: _Position


class MAPFRequest(TypedDict, total=False):
    obstacles: List[_ObstacleEntry]
    tasks: List[_Task]
    mapfile: str
    solver: str
    max_computation_time: int
    max_timestep: int


class MAPFServiceAdapter(MAPFSolverBase):
    """
    TODO: Right now building.yaml files are picked up from the maps directory in the mapf solver service.
    Clean up the process for specifying the map.
    """

    def __init__(self, map_filepath: str, solver_url: str) -> None:
        print("Loading building file:", map_filepath, flush=True)
        self.map_filepath = map_filepath
        self.map = self.load_rmf_map(map_filepath)
        self.solver_url = solver_url

    def generate_uuid(self) -> str:
        return str(uuid.uuid4())

    def load_rmf_map(self, map_filepath: str) -> Dict[str, List[int]]:
        vertices: Dict[str, List[int]] = dict()
        with open(map_filepath) as stream:
            try:
                self.parsed_map = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        if "warehouse" not in self.parsed_map["levels"]:
            return vertices
        if "vertices" not in self.parsed_map["levels"]["warehouse"]:
            return vertices

        for item in self.parsed_map["levels"]["warehouse"]["vertices"]:
            x = int(item[0])
            y = int(item[1])
            vertice_name = item[3]
            vertices[vertice_name] = [x, y]
        return vertices

    def solve(
        self,
        items: List[MAPFAgent],
        input_obstacles: List[Obstacle] = [],
    ) -> List[SolverPlan]:
        logger.debug("Requesting mapf plan..")
        task_ids: Dict[str, str] = {}  # Track task IDs.

        converted_output: MAPFRequest = {}
        tasks: List[_Task] = []
        obstacles: List[_ObstacleEntry] = []
        for item in items:
            robot_id = item.robot_id
            goal_location = self.map[item.goal_location]
            start_location = self.map[item.start_location]
            tasks.append(
                {
                    "agent_name": robot_id,
                    "start_position": {
                        "x": start_location[0],
                        "y": start_location[1],
                    },
                    "end_position": {
                        "x": goal_location[0],
                        "y": goal_location[1],
                    },
                }
            )

            if item.task_id:
                task_ids[item.robot_id] = item.task_id

        if input_obstacles:
            for input_obstacle in input_obstacles:
                location = self.map[input_obstacle.location]
                obstacles.append(
                    {
                        "coordinate": {
                            "x": location[0],
                            "y": location[1],
                        }
                    }
                )
            converted_output["obstacles"] = obstacles

        converted_output["tasks"] = tasks

        # Currently hardcoded elements.
        converted_output["mapfile"] = os.path.basename(self.map_filepath)
        converted_output["solver"] = "ECBS"
        converted_output["max_computation_time"] = 5000
        converted_output["max_timestep"] = 1000
        # print(json.dumps(converted_output, indent=2), flush=True)

        plans_instances: List[SolverPlan] = []

        try:
            plans_json = self.send_mapf_request(converted_output)

            # Add task ID information to each step in solver result
            for p in plans_json:
                for step in p["steps"]:
                    step["task_id"] = task_ids[p["agent_name"]]
                plans_instances.append(SolverPlan.model_validate(p))

        except (
            requests.exceptions.HTTPError,
            requests.exceptions.RequestException,
        ) as e:
            logger.error(f"Request failed: {e}")
            raise

        return plans_instances

    def send_mapf_request(self, json_string_input: MAPFRequest) -> List[Dict[str, Any]]:
        """
        Args:
            json_string_input (MAPFRequest): the MAPF problem payload.

        Returns:
            List[Dict[str, Any]]: the solver's per-agent plans as parsed JSON.
        """

        try:
            logger.debug(f"Posting request to {self.solver_url}")
            response = requests.post(self.solver_url, json=json_string_input)
            logger.debug(f"Response: Code {response.status_code}, Text {response.text}")

            if response.status_code == 200:
                result: List[Dict[str, Any]] = response.json()
                return result
            else:
                response.raise_for_status()
                raise requests.exceptions.RequestException(
                    f"Unexpected status code: {response.status_code}"
                )

        except requests.exceptions.RequestException:
            raise
