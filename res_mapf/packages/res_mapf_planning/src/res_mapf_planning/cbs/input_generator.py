import argparse
import random
from pathlib import Path
from typing import List, Sequence, Tuple

import yaml

from res_mapf_planning.cbs.cbs import AgentContext

"""Program to generate an input.yaml for cbs.py with more agents and obstacles.
"""


def generate_agents(num_agents: int) -> List[AgentContext]:
    """Generate a list of agents"""
    agents: List[AgentContext] = []
    for i in range(num_agents):
        start = [i, 0]
        goal = [i, 0 + 10]
        agent: AgentContext = {"start": start, "goal": goal, "name": f"agent_{i}"}
        agents.append(agent)
    return agents


def generate_obstacles(
    dimensions: Sequence[int], num_obstacles: int
) -> List[Tuple[int, int]]:
    """Generate a list of random obstacle positions."""
    width, height = dimensions
    obstacles: set[Tuple[int, int]] = set()

    while len(obstacles) < num_obstacles:
        obstacle = (random.randint(0, width - 1), random.randint(0, height - 1))
        obstacles.add(obstacle)

    return list(obstacles)


def generate_yaml(
    num_agents: int, num_obstacles: int, map_dimensions: Sequence[int]
) -> str:
    """Generate a YAML string."""

    obstacles = generate_obstacles(map_dimensions, num_obstacles)

    data = {
        "agents": generate_agents(num_agents),
        "map": {"dimensions": map_dimensions, "obstacles": obstacles},
    }

    yaml_string: str = yaml.dump(data, default_flow_style=False, sort_keys=False)
    return yaml_string


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", help="output file")
    args = parser.parse_args()

    num_agents = 200
    num_obstacles = 0
    map_dimensions = [int(num_agents * 1.1), 20]

    # Make sure to modify goals above according to dimensions.

    print(map_dimensions)

    # Generate YAML content
    yaml_content = generate_yaml(num_agents, num_obstacles, map_dimensions)

    file = Path(args.output)
    file.parent.mkdir(parents=True, exist_ok=True)
    # Write YAML content to a file
    with open(args.output, "w") as file:
        file.write(yaml_content)

    print("YAML file generated.")


if __name__ == "__main__":
    main()
