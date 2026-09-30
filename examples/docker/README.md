# MAPF planner Docker example

Minimal stack: RabbitMQ + a planner that consumes AMQP `TaskRequest` messages,
runs CBS on an open 7×7 grid, and serves plans over HTTP.

## Quick run

From this directory:

```bash
docker compose up --build
```

In another terminal, publish tasks over AMQP (host Python with `pika`):

```bash
# against the broker published on localhost:5672
pip install pika   # if needed

# default: two agents head-on (0,0↔2,0) so CBS must resolve conflicts
python amqp_publish_task.py

# single agent instead:
python amqp_publish_task.py --single --robot-id agent_0 --start 0,0 --goal 2,0
```

Expect planner logs: AMQP connected → two `TaskRequest`s → joint CBS plan paths on stdout.

RabbitMQ management UI: <http://localhost:15672> (`guest` / `guest`).

Planner HTTP server: <http://localhost:8090> (port `PLANNER_HTTP_PORT`, default `8090`).

## AMQP message shape

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

## HTTP API

| Method | Path | Description |
| --- | --- | --- |
| `GET` | `/health` | Liveness |
| `GET` | `/plans?source=all\|task\|preview` | Latest stored plans (default `all` = most recent task or preview) |
| `POST` | `/preview` | Dry-run CBS for queued goals (no AMQP, no dispatch) |

### Preview request

```bash
curl -s -X POST http://localhost:8090/preview \
  -H 'Content-Type: application/json' \
  -d '{
    "tasks": [
      {"robot_id": "agent_0", "start": "0,0", "goal_location": "2,0"},
      {"robot_id": "agent_1", "start": "2,0", "goal_location": "0,0"}
    ]
  }'
```

`goal_location` is required. `start` is optional; if omitted, the planner uses the
last known start from a prior AMQP `TaskRequest` for that robot.

Response:

```json
{
  "plans": {
    "agent_0": {
      "robot_id": "agent_0",
      "order_id": "preview_agent_0",
      "map_id": "default_map",
      "plan_version": 0,
      "task_id": "",
      "preview": true,
      "waypoints": [
        {"name": "0,0", "x": 0.0, "y": 0.0, "progress": 0.0, "departure_blockers": []}
      ]
    }
  },
  "error": null
}
```

Fetch stored plans after a task or preview:

```bash
curl -s 'http://localhost:8090/plans'
curl -s 'http://localhost:8090/plans?source=task'
curl -s 'http://localhost:8090/plans?source=preview'
```

## Layout

| Path | Role |
| --- | --- |
| `docker-compose.yml` | `amqp-broker` + `planner` |
| `mapf-planner.Dockerfile` | Builds `planner_app` from [v0.1.0 GitHub Release](https://github.com/GameTL/res_mapf_gametl/releases/tag/v0.1.0) wheels |
| **AMQP** | |
| `amqp_publish_task.py` | AMQP client — publish sample `TaskRequest`(s) |
| `planner_app/amqp_consumer.py` | AMQP transport — consume `TaskRequest` |
| **HTTP server** | |
| `planner_app/http_server.py` | HTTP server — FastAPI `/health`, `/plans`, `/preview` |
| **Shared planner core** | |
| `planner_app/main.py` | Entrypoint — start AMQP thread + HTTP server |
| `planner_app/planner_service.py` | Shared CBS replan + preview (used by both transports) |
| `planner_app/plan_payload.py` | Plan → HTTP/WS payload |
| `planner_app/plan_store.py` | Latest task/preview plan envelopes |
