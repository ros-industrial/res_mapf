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

"""Render a node graph and optional robot schedule in a terminal.

The input may be JSON or YAML and may come from a file or stdin::

    python examples/utils/node_visualiser.py graph.yaml
    some-command | python examples/utils/node_visualiser.py -

Canonical input shape::

    nodes: ["1,5", "2,5"]
    edges: [["1,5", "2,5"]]
    schedule:
      robot-a: [{t: 0, x: 1, y: 5}, {t: 1, x: 2, y: 5}]

Nodes also accept ``[x, y]`` or ``{id: "x,y"}``. A static ``robots`` mapping
may be supplied instead of a schedule. Waiting robots are shown in yellow
(lowercase letter); moving robots use distinct colours. The legend lists
robots and nodes only.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterable, Mapping, Sequence
import json
from pathlib import Path
import re
from string import ascii_uppercase
import sys
from typing import Any

Position = tuple[int, int]
Edge = tuple[Position, Position]
TimedPosition = tuple[int, Position]

_X_SCALE = 4
_Y_SCALE = 2

_RESET = "\033[0m"
_YELLOW = "\033[33m"
_RED = "\033[31m"
_DIM = "\033[2m"
_ROBOT_COLORS = (
    "\033[36m",  # cyan
    "\033[32m",  # green
    "\033[35m",  # magenta
    "\033[34m",  # blue
    "\033[91m",  # bright red
    "\033[96m",  # bright cyan
)


def _color(text: str, code: str) -> str:
    return f"{code}{text}{_RESET}"


def _position(value: Any) -> Position:
    if isinstance(value, str):
        x, y = value.split(",", maxsplit=1)
        return int(x), int(y)
    if isinstance(value, Mapping):
        if "id" in value:
            return _position(value["id"])
        if "position" in value:
            return _position(value["position"])
        return int(value["x"]), int(value["y"])
    if isinstance(value, Sequence) and len(value) == 2:
        return int(value[0]), int(value[1])
    raise ValueError(f"cannot read graph position from {value!r}")


def _edge(value: Any) -> Edge:
    if isinstance(value, Mapping):
        start = value.get("from", value.get("start"))
        end = value.get("to", value.get("end"))
    else:
        start, end = value
    return _position(start), _position(end)


def _schedule(value: Mapping[str, Any]) -> dict[str, list[TimedPosition]]:
    result: dict[str, list[TimedPosition]] = {}
    for robot, raw_path in value.items():
        path: list[TimedPosition] = []
        for default_time, state in enumerate(raw_path):
            if isinstance(state, Mapping):
                time = int(state.get("t", default_time))
                position = _position(state)
            else:
                time = default_time
                position = _position(state)
            path.append((time, position))
        if not path:
            raise ValueError(f"robot {robot!r} has an empty schedule")
        result[str(robot)] = path
    return result


def _position_at(path: Sequence[TimedPosition], time: int) -> Position:
    for state_time, position in reversed(path):
        if state_time <= time:
            return position
    return path[0][1]


def _put(canvas: list[list[str]], row: int, column: int, char: str) -> None:
    current = canvas[row][column]
    canvas[row][column] = char if current in {" ", char} else "┼"


def _draw_edge(
    canvas: list[list[str]], start: tuple[int, int], end: tuple[int, int]
) -> None:
    start_row, start_column = start
    end_row, end_column = end
    row_distance = end_row - start_row
    column_distance = end_column - start_column
    steps = max(abs(row_distance), abs(column_distance))
    if not steps:
        return

    if not row_distance:
        char = "─"
    elif not column_distance:
        char = "│"
    elif row_distance * column_distance < 0:
        char = "╱"
    else:
        char = "╲"

    for step in range(1, steps):
        row = round(start_row + row_distance * step / steps)
        column = round(start_column + column_distance * step / steps)
        _put(canvas, row, column, char)


def render_graph(
    nodes: Iterable[Any],
    edges: Iterable[Any],
    robots: Mapping[str, Any] | None = None,
    waiting: Iterable[str] = (),
) -> str:
    """Return one terminal frame from flexible node, edge, and robot inputs."""
    graph_nodes = {_position(node) for node in nodes}
    graph_edges = [_edge(edge) for edge in edges]
    robot_positions = {
        str(robot): _position(position) for robot, position in (robots or {}).items()
    }
    graph_nodes.update(endpoint for edge in graph_edges for endpoint in edge)
    graph_nodes.update(robot_positions.values())
    if not graph_nodes:
        return "(empty graph)"
    if len(robot_positions) > len(ascii_uppercase):
        raise ValueError("at most 26 robots can be displayed")

    min_x = min(x for x, _ in graph_nodes)
    max_x = max(x for x, _ in graph_nodes)
    min_y = min(y for _, y in graph_nodes)
    max_y = max(y for _, y in graph_nodes)
    canvas = [
        [" " for _ in range((max_x - min_x) * _X_SCALE + 1)]
        for _ in range((max_y - min_y) * _Y_SCALE + 1)
    ]

    def canvas_position(position: Position) -> tuple[int, int]:
        x, y = position
        return (max_y - y) * _Y_SCALE, (x - min_x) * _X_SCALE

    for start, end in graph_edges:
        _draw_edge(canvas, canvas_position(start), canvas_position(end))
    for node in graph_nodes:
        row, column = canvas_position(node)
        canvas[row][column] = _color("·", _DIM)

    waiting_robots = set(waiting)
    symbols = {
        robot: ascii_uppercase[index] for index, robot in enumerate(robot_positions)
    }
    colors = {
        robot: _ROBOT_COLORS[index % len(_ROBOT_COLORS)]
        for index, robot in enumerate(robot_positions)
    }
    occupied: set[Position] = set()
    for robot, position in robot_positions.items():
        row, column = canvas_position(position)
        if position in occupied:
            marker = _color("!", _RED)
        elif robot in waiting_robots:
            marker = _color(symbols[robot].lower(), _YELLOW)
        else:
            marker = _color(symbols[robot], colors[robot])
        canvas[row][column] = marker
        occupied.add(position)

    drawing = "\n".join("".join(row).rstrip() for row in canvas)
    if not symbols:
        return drawing
    legend_parts = [
        _color(f"{symbol}={robot}", colors[robot]) for robot, symbol in symbols.items()
    ]
    legend = "  ".join(legend_parts)
    return f"{drawing}\n{legend}  {_color('·', _DIM)}=node"


_ANSI_RE = re.compile(r"\033\[[0-9;]*m")
_DIVIDER = "--------------------------------"
_PANEL_GAP = "   |   "


def _visible_len(text: str) -> int:
    return len(_ANSI_RE.sub("", text))


def _pad_visible(text: str, width: int) -> str:
    return text + " " * max(0, width - _visible_len(text))


def _side_by_side(left: str, right: str, gap: str = _PANEL_GAP) -> str:
    """Place two multi-line panels next to each other with a padded divider."""
    left_lines = left.splitlines() or [""]
    right_lines = right.splitlines() or [""]
    left_width = max(_visible_len(line) for line in left_lines)
    height = max(len(left_lines), len(right_lines))
    rows: list[str] = []
    for index in range(height):
        left_line = left_lines[index] if index < len(left_lines) else ""
        right_line = right_lines[index] if index < len(right_lines) else ""
        rows.append(f"{_pad_visible(left_line, left_width)}{gap}{right_line}")
    return "\n".join(rows)


def render_timeline(
    nodes: Iterable[Any], edges: Iterable[Any], schedule: Mapping[str, Any]
) -> str:
    """Return start/goal summary, then all schedule frames.

    Final positions are held until the makespan. Start and goal are shown
    side by side first, then the timed frames.
    """
    paths = _schedule(schedule)
    if not paths:
        return "(empty schedule)"
    graph_nodes = list(nodes)
    graph_edges = list(edges)
    makespan = max(time for path in paths.values() for time, _ in path)

    start = {robot: path[0][1] for robot, path in paths.items()}
    goal = {robot: path[-1][1] for robot, path in paths.items()}
    start_drawing, start_legend = render_graph(graph_nodes, graph_edges, start).rsplit(
        "\n", 1
    )
    goal_drawing, _ = render_graph(graph_nodes, graph_edges, goal).rsplit("\n", 1)
    start_panel = f"start\n{start_drawing}"
    goal_panel = f"goal\n{goal_drawing}"
    summary = f"{_side_by_side(start_panel, goal_panel)}\n{start_legend}"

    frames: list[str] = []
    previous: dict[str, Position] = {}
    for time in range(makespan + 1):
        current = {robot: _position_at(path, time) for robot, path in paths.items()}
        waiting = {
            robot
            for robot, position in current.items()
            if previous.get(robot) == position and position != goal[robot]
        }
        frames.append(
            f"t={time}\n{render_graph(graph_nodes, graph_edges, current, waiting)}"
        )
        previous = current

    return f"{summary}\n{_DIVIDER}\n\n" + "\n\n".join(frames)


def _load_input(path: str) -> Mapping[str, Any]:
    text = sys.stdin.read() if path == "-" else Path(path).read_text()
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        try:
            import yaml
        except ImportError as error:
            raise ValueError("input is not JSON and PyYAML is not installed") from error
        data = yaml.safe_load(text)
    if not isinstance(data, Mapping):
        raise ValueError("input must be a JSON/YAML object")
    return data


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", nargs="?", default="-", help="JSON/YAML file, or - for stdin"
    )
    args = parser.parse_args()
    data = _load_input(args.input)
    nodes = data.get("nodes", ())
    edges = data.get("edges", ())
    schedule = data.get("schedule", data.get("plan"))
    if schedule is not None:
        print(render_timeline(nodes, edges, schedule))
    else:
        print(render_graph(nodes, edges, data.get("robots")))


if __name__ == "__main__":
    main()
