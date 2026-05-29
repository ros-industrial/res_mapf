# MIT License

# Copyright (c) 2019 Ashwin Bose

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

import argparse
from typing import Dict, List, Sequence, TypedDict

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib import animation
from matplotlib.artist import Artist

# matplotlib.use("Agg")
from matplotlib.patches import Circle, Patch, Rectangle
from matplotlib.text import Text

from res_mapf_planning.cbs.cbs import AgentContext, CBSPathState

Colors = ["orange", "blue", "green"]


class MapSection(TypedDict):
    dimensions: Sequence[int]
    obstacles: Sequence[Sequence[int]]


class MapInput(TypedDict):
    map: MapSection
    agents: Sequence[AgentContext]


class ScheduleInput(TypedDict):
    schedule: Dict[str, Sequence[CBSPathState]]


class Animation:
    def __init__(self, map: MapInput, schedule: ScheduleInput) -> None:
        self.map = map
        self.schedule = schedule
        self.combined_schedule: Dict[str, Sequence[CBSPathState]] = {}
        self.combined_schedule.update(self.schedule["schedule"])

        aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]

        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect="equal")
        self.fig.subplots_adjust(
            left=0, right=1, bottom=0, top=1, wspace=None, hspace=None
        )
        # self.ax.set_frame_on(False)

        self.patches: List[Patch] = []
        self.artists: List[Artist] = []
        self.agents: Dict[str, Circle] = dict()
        self.agent_names: Dict[str, Text] = dict()
        self.original_face_colors: Dict[str, str] = dict()
        # create boundary patch
        xmin = -0.5
        ymin = -0.5
        xmax = map["map"]["dimensions"][0] - 0.5
        ymax = map["map"]["dimensions"][1] - 0.5

        # self.ax.relim()
        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)
        # self.ax.set_xticks([])
        # self.ax.set_yticks([])
        # plt.axis('off')
        # self.ax.axis('tight')
        # self.ax.axis('off')

        self.patches.append(
            Rectangle(
                (xmin, ymin),
                xmax - xmin,
                ymax - ymin,
                facecolor="none",
                edgecolor="red",
            )
        )
        for o in map["map"]["obstacles"]:
            x, y = o[0], o[1]
            self.patches.append(
                Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor="red", edgecolor="red")
            )

        # create agents:
        self.T = 0
        # draw goals first
        for d, i in zip(map["agents"], range(0, len(map["agents"]))):
            self.patches.append(
                Rectangle(
                    (d["goal"][0] - 0.25, d["goal"][1] - 0.25),
                    0.5,
                    0.5,
                    facecolor=Colors[0],
                    edgecolor="black",
                    alpha=0.5,
                )
            )
        for d, i in zip(map["agents"], range(0, len(map["agents"]))):
            name = d["name"]
            self.agents[name] = Circle(
                (d["start"][0], d["start"][1]),
                0.3,
                facecolor=Colors[0],
                edgecolor="black",
            )
            self.original_face_colors[name] = Colors[0]
            self.patches.append(self.agents[name])
            self.T = max(self.T, schedule["schedule"][name][-1]["t"])
            self.agent_names[name] = self.ax.text(
                d["start"][0], d["start"][1], name.replace("agent", "")
            )
            self.agent_names[name].set_horizontalalignment("center")
            self.agent_names[name].set_verticalalignment("center")
            self.artists.append(self.agent_names[name])

        # self.ax.set_axis_off()
        # self.fig.axes[0].set_visible(False)
        # self.fig.axes.get_yaxis().set_visible(False)

        # self.fig.tight_layout()

        self.anim = animation.FuncAnimation(
            self.fig,
            self.animate_func,
            init_func=self.init_func,
            frames=int(self.T + 1) * 10,
            interval=100,
            blit=True,
        )

    def save(self, file_name: str, speed: int) -> None:
        self.anim.save(file_name, "ffmpeg", fps=10 * speed, dpi=200)
        # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

    def show(self) -> None:
        plt.show()

    def init_func(self) -> List[Artist]:
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return [*self.patches, *self.artists]

    def animate_func(self, i: int) -> List[Artist]:
        for agent_name, agent in self.combined_schedule.items():
            pos = self.getState(i / 10, agent)
            p = (float(pos[0]), float(pos[1]))
            self.agents[agent_name].center = p
            self.agent_names[agent_name].set_position(p)

        # reset all colors
        for name, circle in self.agents.items():
            circle.set_facecolor(self.original_face_colors[name])

        # check drive-drive collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor("red")
                    d2.set_facecolor("red")
                    print("COLLISION! (agent-agent) ({}, {})".format(i, j))

        return [*self.patches, *self.artists]

    def getState(self, t: float, d: Sequence[CBSPathState]) -> "np.ndarray":
        idx = 0
        while idx < len(d) and d[idx]["t"] < t:
            idx += 1
        if idx == 0:
            first: np.ndarray = np.array([float(d[0]["x"]), float(d[0]["y"])])
            return first
        elif idx < len(d):
            posLast: np.ndarray = np.array(
                [float(d[idx - 1]["x"]), float(d[idx - 1]["y"])]
            )
            posNext: np.ndarray = np.array([float(d[idx]["x"]), float(d[idx]["y"])])
        else:
            last: np.ndarray = np.array([float(d[-1]["x"]), float(d[-1]["y"])])
            return last
        dt = d[idx]["t"] - d[idx - 1]["t"]
        t = (t - d[idx - 1]["t"]) / dt
        pos: np.ndarray = (posNext - posLast) * t + posLast
        return pos


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map")
    parser.add_argument("schedule", help="schedule for agents")
    parser.add_argument(
        "--video",
        dest="video",
        default=None,
        help="output video file (or leave empty to show on screen)",
    )
    parser.add_argument("--speed", type=int, default=1, help="speedup-factor")
    args = parser.parse_args()

    with open(args.map) as map_file:
        map = yaml.load(map_file, Loader=yaml.FullLoader)

    with open(args.schedule) as states_file:
        schedule = yaml.load(states_file, Loader=yaml.FullLoader)

    anim = Animation(map, schedule)

    if args.video:
        anim.save(args.video, args.speed)
    else:
        anim.show()
