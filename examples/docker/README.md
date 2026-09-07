# MAPF planner Docker example

Minimal stack: RabbitMQ + a planner that consumes AMQP `TaskRequest` messages,
runs CBS on an open 7×7 grid, and logs the plan to stdout.

Nothing is published back to AMQP. There is no VDA5050 / HTTP master.

## Quick run

From this directory:

```bash
docker compose up --build
```

In another terminal, publish a task (host Python with `pika`, or any AMQP client):

```bash
# against the broker published on localhost:5672
pip install pika   # if needed
python publish_task.py
# or:
python publish_task.py --robot-id agent_0 --start 0,0 --goal 2,0
python publish_task.py --robot-id agent_1 --task-id t2 --start 2,0 --goal 0,0
```

Expect planner logs: AMQP connected → `TaskRequest` → CBS plan -> ROS 2 Plan message displayed on stdout.

RabbitMQ management UI: <http://localhost:15672> (`guest` / `guest`).

## Message shape

```json
{
  "type": "TaskRequest",
  "robot_id": "agent_0",
  "task_id": "t1",
  "start": "0,0",
  "goal": "2,0"
}
```

Grid cells are `"x,y"` integers with `0 <= x,y <= 6` (CBSAdapter open grid).

## Layout

| Path | Role |
| --- | --- |
| `docker-compose.yml` | `amqp-broker` + `planner` |
| `mapf-planner.Dockerfile` | Builds `planner_app` from [v0.1.0 GitHub Release](https://github.com/GameTL/res_mapf_gametl/releases/tag/v0.1.0) wheels |
| `planner_app/` | Long-running consumer |
| `publish_task.py` | One-shot TaskRequest publisher for smoke tests |
