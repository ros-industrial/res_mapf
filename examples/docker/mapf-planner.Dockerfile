# MAPF planner example — AMQP consumer + HTTP server (CBS plans).
FROM ghcr.io/astral-sh/uv:python3.12-bookworm-slim

WORKDIR /app

COPY examples/docker/planner_app/ /app/examples/docker/planner_app/

WORKDIR /app/examples/docker/planner_app

RUN uv sync --no-dev

ENV PYTHONUNBUFFERED=1

CMD ["uv", "run", "--no-sync", "main.py"]
