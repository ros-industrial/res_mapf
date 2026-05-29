Plan Server: Receive destination requests. Tracks comitted positions.
MAPFCoordinator: Bridge Plan Server and MAPFSolverABC. Translates current system state and tasks into solvable MAPF problem. 
MultiAgentContext: Keeps track of which agents exist, their current and pending goals.
MAPFSolverABC: interface for classical MAPF solvers. Given starts, goals and obstacles, find paths.
