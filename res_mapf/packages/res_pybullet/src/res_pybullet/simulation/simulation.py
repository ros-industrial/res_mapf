# MIT License

# Copyright (c) 2022 OMRON SINIC X Corporation

# Author: Kazumi Kasaura

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


"""
Runs the PyBullet simulation.
Robots are initialised and then continuously receive commands over the AgentCommand interface,
responding on completion of each command.

Provide a correct building.yaml to use named waypoints.

This simulation.py and related urdf files were adapted initially from:
    https://github.com/omron-sinicx/PSIPP-CTC/tree/master/tools under the MIT License
    Copyright (c) 2022 OMRON SINIC X Corporation
    Author: Kazumi Kasaura
"""

import argparse
import math
import time
from pathlib import Path
from typing import Any, Optional, Dict, Sequence, Tuple

import numpy
import pybullet as p  # type: ignore
import quaternion  # type: ignore
import yaml
from res_pybullet.agent.agent import Agent, AgentCommandInterface, AgentState
from res_pybullet.utils.pybullet_helpers import (
    check_escape_key,
    draw_grid,
    set_camera_view,
)

URDF_DIRECTORY = str(Path(__file__).parent / "assets")

# Simulation parameters -------
dt = 1 / 240.0  # default bullet step
MAX_FORCE = 50

# Robot velocity and acceleration constants
V_VEHICLE = 1.0  # target linear speed
TIME_TO_ACCELERATE = 0.4  # desired time to accelerate to target speed
WHEEL_VEL_FOR_TURNING = 6.0  # wheel angular velocity to use when turning on the spot
WHEEL_RADIUS = 0.05
# Derived values
MAX_W_WHEEL = V_VEHICLE / WHEEL_RADIUS  # maximum allowed wheel angular velocity
W_WHEEL_INCREMENT = MAX_W_WHEEL / (TIME_TO_ACCELERATE / dt)
# -----------------------------


def load_building(map_filepath: str) -> Optional[Dict[str, Sequence[float]]]:
    try:
        vertices: Dict[str, Sequence[float]] = {}
        with open(map_filepath) as stream:
            print(f"Loading map {map_filepath}")
            try:
                parsed_map = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return None

        if "warehouse" not in parsed_map["levels"]:
            return vertices
        if "vertices" not in parsed_map["levels"]["warehouse"]:
            return vertices

        for item in parsed_map["levels"]["warehouse"]["vertices"]:
            x = float(item[0])
            y = float(item[1])
            vertice_name = item[3]
            vertices[vertice_name] = [x, y]
        return vertices
    except IOError:
        return None


def parse_coords(coord_string: str) -> Sequence[Tuple[float, float]]:
    """
    Parse a string of space-separated coordinates into a list of tuples.
    Example: "1,2 3,4" -> [(1,2), (3,4)]
    """
    coords = []
    for pair in coord_string.split():
        x_str, y_str = pair.split(",")
        coords.append((float(x_str), float(y_str)))
    return coords


def find_theta_diff(
    target: Sequence[float], position: Sequence[float], direction: Sequence[float]
) -> float:
    theta_diff = math.atan2(
        target[1] - position[1], target[0] - position[0]
    ) - math.atan2(direction[1], direction[0])
    if theta_diff > numpy.pi:
        theta_diff -= 2 * numpy.pi
    if theta_diff < -numpy.pi:
        theta_diff += 2 * numpy.pi
    return theta_diff


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--hide-labels", action="store_true", help="Disable debug text."
    )
    parser.add_argument(
        "--building",
        help="Path to building.yaml. A correct file must be provided if you want to use named waypoints.",
    )
    parser.add_argument("--video", action="store_true", help="Records a video.")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--named",
        nargs="+",
        help="List of named waypoints for start locations for robots.",
    )
    group.add_argument(
        "--coords",
        type=parse_coords,
        help='Space-separated list of coordinates, e.g. "1,2 3,4" for start locations for robots.',
    )

    args = parser.parse_args()

    def wrap_debug_text(*wrap_args: Any, **wrap_kwargs: Any) -> Any:
        if not args.hide_labels:
            return p.addUserDebugText(*wrap_args, **wrap_kwargs)

    vertices = None
    if args.building:
        vertices = load_building(args.building)
        if vertices:
            print("Loaded building map")
            x_max = max([vertices[key][0] for key in vertices])
            x_min = min([vertices[key][0] for key in vertices])
            y_max = max([vertices[key][1] for key in vertices])
            y_min = min([vertices[key][1] for key in vertices])
            print(f"{x_max}, {x_min}, {y_max}, {y_min}")

    # Set up the simulation
    p.connect(p.GUI)
    p.loadURDF(URDF_DIRECTORY + "/plane.urdf")
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    # Initialise debug
    # colors = plt.get_cmap(name="hsv", lut=number_of_agents + 1)
    # Initialise debug item IDs
    bullet_debug_items = {}

    camera_target_x = 0
    camera_target_y = 0
    simulation_agents = {}

    time_start = time.time()

    num_agents = len(args.named) if args.named else len(args.coords)

    for i in range(num_agents):
        # Spawn agent at starting location
        agent_name = f"agent_{i}"

        if args.named:
            # Use named waypoint (requires vertices map)
            name = args.named[i]
            if vertices and name in vertices:
                x, y = vertices[name]
            else:
                raise ValueError(f"Named waypoint '{name}' not found in vertices map.")
        else:
            # Use explicit coordinate
            x, y = args.coords[i]
        print(f"Spawning agent at {x}, {y}")
        # TODO: Investigate effect of URDF_MERGE_FIXED_LINKS
        # simulation_agents[plan.agent_name]= (Agent(p.loadURDF(URDF_DIRECTORY + "/toio.urdf", (x, y, 0.1), flags=p.URDF_MERGE_FIXED_LINKS)))
        simulation_agents[agent_name] = Agent(
            p.loadURDF(URDF_DIRECTORY + "/toio.urdf", (x, y, 0.1))
        )

        # Save coordinates to point camera at
        camera_target_x = int(x)
        camera_target_y = int(y)

    time_end = time.time()
    print(f"Time taken to spawn agents: {time_end - time_start}")

    # set_camera_view(p, num_agents=0, xmin=x_min, xmax=x_max, ymin=y_min, ymax=y_max)
    set_camera_view(p, camera_target_x, camera_target_y)

    # Always draw the default grid
    draw_grid(p, 0, 6, 0, 6, 1)  # TODO dynamic grid size and camera view

    # Additionally, draw grid for the loaded building.
    if vertices:
        draw_grid(
            p, int(x_min), int(x_max), int(y_min), int(y_max), 10
        )  # TODO dynamic grid size and camera view

    bullet_debug_items["timestamp"] = wrap_debug_text("", [0, 0, 0])

    for agent_name in simulation_agents:
        bullet_debug_items[agent_name + "_position_label"] = wrap_debug_text(
            "", [0, 0, 0]
        )
        bullet_debug_items[agent_name + "_state_label"] = wrap_debug_text(
            agent_name, [0, 0, 0]
        )

    time.sleep(1)
    living = True
    current_time = 0.0

    video_logger_id = 0
    if args.video:
        video_logger_id = p.startStateLogging(
            p.STATE_LOGGING_VIDEO_MP4, "pybullet_video.mp4"
        )

    for i in range(100):
        p.stepSimulation()

    agent_cmd_ifaces = {}
    for agent_name in simulation_agents:
        agent_cmd_ifaces[agent_name] = AgentCommandInterface(
            agent_name, vertices_map=vertices
        )

    for agent_name, current_agent in simulation_agents.items():
        wrap_debug_text(
            agent_name + " " + current_agent.agent_state.name,
            textPosition=(0, 0, 0.2),
            parentObjectUniqueId=current_agent.id,
            textColorRGB=current_agent.agent_state.value,
            replaceItemUniqueId=bullet_debug_items[agent_name + "_state_label"],
        )

    step = 0
    while living:
        p.stepSimulation()
        time.sleep(dt)
        step += 1
        current_time += dt
        if step % 48 == 0:
            wrap_debug_text(
                "%.3f" % (current_time),
                [0, 0, 0],
                replaceItemUniqueId=bullet_debug_items["timestamp"],
            )

        # Check for collision in previously computed contact points
        # contact_points = p.getContactPoints()
        # collision = False
        # for contact_point in contact_points:
        #     if contact_point[1] != plane and contact_point[2] != plane:
        #         i = contact_point[1] - 1
        #         j = contact_point[2] - 1
        #         position_i = p.getBasePositionAndOrientation(simulation_agents[i].id)[0]
        #         position_j = p.getBasePositionAndOrientation(simulation_agents[j].id)[0]
        #         print(position_i, position_j)
        #         wrap_debug_text(
        #             "Collision!",
        #             [position_i[0], position_j[1], 0],
        #             replaceItemUniqueId=bullet_debug_items["timestamp"],
        #         )
        #         collision = True
        #         break
        # if collision:
        #     print("collision!")

        if check_escape_key(p):
            p.stopStateLogging(video_logger_id)
            break

        for agent_name, current_agent in simulation_agents.items():
            if simulation_agents[agent_name].is_reached_goal():
                agent_state = AgentState.COMPLETED
                p.setJointMotorControl2(
                    current_agent.id,
                    0,
                    p.VELOCITY_CONTROL,
                    targetVelocity=0,
                    force=MAX_FORCE,
                )
                p.setJointMotorControl2(
                    current_agent.id,
                    1,
                    p.VELOCITY_CONTROL,
                    targetVelocity=0,
                    force=MAX_FORCE,
                )

            else:
                current_controller = agent_cmd_ifaces[agent_name]
                agent_state = AgentState.MOVING

                state = p.getBasePositionAndOrientation(current_agent.id)
                position = state[0][:2]
                quat = quaternion.quaternion(state[1][3], state[1][0], state[1][1], state[1][2])
                direction = quaternion.rotate_vectors(quat, (1.0, 0.0, 0.0))[:2]
                targ, targ_id, subsequent_targ, is_goal = (
                    current_controller.get_target()
                )
                if not targ:
                    agent_state = AgentState.WAITING
                    p.setJointMotorControl2(
                        current_agent.id,
                        0,
                        p.VELOCITY_CONTROL,
                        targetVelocity=0,
                        force=MAX_FORCE,
                    )
                    p.setJointMotorControl2(
                        current_agent.id,
                        1,
                        p.VELOCITY_CONTROL,
                        targetVelocity=0,
                        force=MAX_FORCE,
                    )
                    if agent_state != current_agent.agent_state:
                        current_agent.agent_state = agent_state
                        wrap_debug_text(
                            agent_name + " " + agent_state.name,
                            textPosition=(0, 0, 0.2),
                            parentObjectUniqueId=current_agent.id,
                            textColorRGB=agent_state.value,
                            replaceItemUniqueId=bullet_debug_items[
                                agent_name + "_state_label"
                            ],
                        )
                    continue
                target = numpy.asarray(targ)
                # print("     position: ", position, " target:", target, " diff: ", numpy.linalg.norm(target-position))
                if numpy.linalg.norm(target - position) < 0.01:
                    agent_state = AgentState.REACHED_WP
                    current_controller.completed(targ_id)
                    wrap_debug_text(
                        agent_name + " " + agent_state.name,
                        textPosition=(0, 0, 0.2),
                        parentObjectUniqueId=current_agent.id,
                        textColorRGB=agent_state.value,
                        replaceItemUniqueId=bullet_debug_items[
                            agent_name + "_state_label"
                        ],
                    )

                theta_diff = find_theta_diff(target, position, direction)

                if abs(theta_diff) < 0.15 and abs(theta_diff) > 0.1:
                    v_rot = WHEEL_VEL_FOR_TURNING * (1 - abs(2 * theta_diff))
                else:
                    v_rot = WHEEL_VEL_FOR_TURNING

                if theta_diff > 0.1:
                    agent_state = AgentState.ROTATING
                    p.setJointMotorControl2(
                        current_agent.id,
                        0,
                        p.VELOCITY_CONTROL,
                        targetVelocity=v_rot,
                        force=MAX_FORCE,
                    )
                    p.setJointMotorControl2(
                        current_agent.id,
                        1,
                        p.VELOCITY_CONTROL,
                        targetVelocity=-v_rot,
                        force=MAX_FORCE,
                    )
                elif theta_diff < -0.1:
                    agent_state = AgentState.ROTATING
                    p.setJointMotorControl2(
                        current_agent.id,
                        0,
                        p.VELOCITY_CONTROL,
                        targetVelocity=-v_rot,
                        force=MAX_FORCE,
                    )
                    p.setJointMotorControl2(
                        current_agent.id,
                        1,
                        p.VELOCITY_CONTROL,
                        targetVelocity=v_rot,
                        force=MAX_FORCE,
                    )
                else:
                    xyzlinvel = p.getBaseVelocity(current_agent.id)[0]
                    linvel = math.sqrt(
                        math.pow(xyzlinvel[0], 2) + math.pow(xyzlinvel[1], 2)
                    )

                    if linvel < 0.99:
                        targetvel = min(
                            MAX_W_WHEEL, current_agent.last_vel_cmd + W_WHEEL_INCREMENT
                        )
                        current_agent.last_vel_cmd = targetvel
                    # If the subsequent target is not in the same direction, slow down
                    if subsequent_targ:
                        theta_diff_subsequent = find_theta_diff(
                            subsequent_targ, position, direction
                        )
                    else:
                        theta_diff_subsequent = 0.0
                    if (abs(theta_diff_subsequent) > 0.2) and (
                        numpy.linalg.norm(target - position) < 0.5
                    ):
                        targetvel = min(
                            MAX_W_WHEEL / 2,
                            current_agent.last_vel_cmd - W_WHEEL_INCREMENT,
                        )
                        current_agent.last_vel_cmd = targetvel

                    modifier = 0.5
                    if theta_diff > 0.01:
                        agent_state = AgentState.MOVING_R
                        p.setJointMotorControl2(
                            current_agent.id,
                            0,
                            p.VELOCITY_CONTROL,
                            targetVelocity=(1 + modifier) * targetvel,
                            force=MAX_FORCE,
                        )
                        p.setJointMotorControl2(
                            current_agent.id,
                            1,
                            p.VELOCITY_CONTROL,
                            targetVelocity=(1 - modifier) * targetvel,
                            force=MAX_FORCE,
                        )
                    elif theta_diff < -0.01:
                        agent_state = AgentState.MOVING_R
                        p.setJointMotorControl2(
                            current_agent.id,
                            0,
                            p.VELOCITY_CONTROL,
                            targetVelocity=(1 - modifier) * targetvel,
                            force=MAX_FORCE,
                        )
                        p.setJointMotorControl2(
                            current_agent.id,
                            1,
                            p.VELOCITY_CONTROL,
                            targetVelocity=(1 + modifier) * targetvel,
                            force=MAX_FORCE,
                        )
                    else:
                        p.setJointMotorControl2(
                            current_agent.id,
                            0,
                            p.VELOCITY_CONTROL,
                            targetVelocity=targetvel,
                            force=MAX_FORCE,
                        )
                        p.setJointMotorControl2(
                            current_agent.id,
                            1,
                            p.VELOCITY_CONTROL,
                            targetVelocity=targetvel,
                            force=MAX_FORCE,
                        )
                if not args.hide_labels:
                    if step % 48 == 0:
                        # Note that addUserDebugText drastically slows down the simulation if added at every step
                        xyzlinvel = p.getBaseVelocity(current_agent.id)[0]
                        linvel = math.sqrt(
                            math.pow(xyzlinvel[0], 2) + math.pow(xyzlinvel[1], 2)
                        )
                        show_robot_speed_and_position = False
                        if show_robot_speed_and_position:
                            wrap_debug_text(
                                "%.1f" % (position[0])
                                + ", "
                                + "%.1f" % (position[1])
                                + ", "
                                + "Speed: %.1f" % (linvel),
                                textPosition=(0, 0, 0.3),
                                parentObjectUniqueId=current_agent.id,
                                textColorRGB=agent_state.value,
                                replaceItemUniqueId=bullet_debug_items[
                                    agent_name + "_position_label"
                                ],
                            )
                    if agent_state != current_agent.agent_state:
                        current_agent.agent_state = agent_state
                        wrap_debug_text(
                            agent_name + " " + agent_state.name,
                            textPosition=(0, 0, 0.2),
                            parentObjectUniqueId=current_agent.id,
                            textColorRGB=agent_state.value,
                            replaceItemUniqueId=bullet_debug_items[
                                agent_name + "_state_label"
                            ],
                        )


if __name__ == "__main__":
    main()
