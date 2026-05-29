from typing import Sequence, Tuple

import pytest

from res_mapf_planning.cbs.cbs import CBS, AgentContext, Environment
from res_mapf_planning.cbs.visualize import Animation, MapInput, ScheduleInput


@pytest.mark.timeout(2)
@pytest.mark.parametrize(
    "dimension, agents, obstacles",
    [
        #   +---+---+---+---+---+
        # 4 |   |   |   |   |   |
        #   +---+---+---+---+---+
        # 3 |   |   |   |   |   |
        #   +---+---+---+---+---+
        # 2 |   |   |   |   |   |
        #   +---+---+---+---+---+
        # 1 | # |   | # |   |   |
        #   +---+---+---+---+---+
        # 0 | A | A | A |   | A |
        #   +---+---+---+---+---+
        #     0   1   2   3   4
        (
            [5, 5],
            [
                {"start": (0, 0), "goal": (0, 2), "name": "0"},
                {"start": (1, 0), "goal": (1, 3), "name": "1"},
                {"start": (2, 0), "goal": (3, 0), "name": "2"},
                {"start": (4, 0), "goal": (4, 4), "name": "3"},
            ],
            [(0, 1), (2, 1)],
        ),
        #   +---+---+---+---+---+
        # 4 |   |   |   |   |   |
        #   +---+---+---+---+---+
        # 3 |   |   |   |   |   |
        #   +---+---+---+---+---+
        # 2 | a |   | b |   |   |
        #   +---+---+---+---+---+
        # 1 | # |   | # |   |   |
        #   +---+---+---+---+---+
        # 0 | A |   | B | # |   |
        #   +---+---+---+---+---+
        #     0   1   2   3   4
        (
            [5, 5],
            [
                {"start": (0, 0), "goal": (0, 2), "name": "A"},
                {"start": (2, 0), "goal": (2, 2), "name": "B"},
            ],
            [(0, 1), (2, 1), (3, 0)],
        ),
        #   +---+---+---+---+---+
        # 4 |   |   |   |   |   |
        #   +---+---+---+---+---+
        # 3 |   |   |   |   |   |
        #   +---+---+---+---+---+
        # 2 |   | # |   |   |   |
        #   +---+---+---+---+---+
        # 1 | # |   | # |   |   |
        #   +---+---+---+---+---+
        # 0 | A | B |   | # |   |
        #   +---+---+---+---+---+
        #     0   1   2   3   4
        (
            [5, 5],
            [
                {"start": (0, 0), "goal": (2, 0), "name": "A"},
                {"start": (1, 0), "goal": (1, 0), "name": "B"},
            ],
            [(0, 1), (1, 2), (2, 1), (3, 0)],
        ),
    ],
)
def test_scenarios(
    dimension: Sequence[int],
    agents: Sequence[AgentContext],
    obstacles: Sequence[Tuple[int, int]],
) -> None:
    env = Environment(dimension, agents, obstacles)

    # Searching
    cbs = CBS(env)
    solution = cbs.search()

    input_data: MapInput = {
        "agents": agents,
        "map": {"dimensions": dimension, "obstacles": obstacles},
    }
    schedule_input: ScheduleInput = {"schedule": solution}

    assert env.compute_solution_cost(solution) >= 0

    animation = Animation(input_data, schedule_input)

    show_animations = False  # Set to True to visualise with GUI.
    if show_animations:
        animation.show()
