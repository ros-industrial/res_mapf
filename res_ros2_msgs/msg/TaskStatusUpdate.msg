# TaskStatusUpdate.msg

# Task ID
string task_id

# Robot ID
string robot_id

# TaskStatus
TaskStatus status

# "plan_server" | "plan_executor"
string source

# Empty string if none
string reason

# Empty string if none
string superseded_by

builtin_interfaces/Time timestamp
