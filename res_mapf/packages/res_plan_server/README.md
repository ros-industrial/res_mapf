| Component          | Responsibility |
|--------------------|----------------|
| Plan Server        | Receive destination requests. Tracks committed positions. |
| MAPFCoordinator    | Bridge Plan Server and MAPFSolverABC. Translates current system state and tasks into a solvable MAPF problem. |
| MultiAgentContext  | Keeps track of which agents exist, their current and pending goals. |
| MAPFSolverBase     | Interface for classical MAPF solvers. Given starts, goals, and obstacles, finds paths. |
