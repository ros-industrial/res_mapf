### Usage

Build
```
cd ~/mapf_ws
source ~/colcon_extra_ws/install/setup.sh; colcon venv sync; 
colcon build
```

#### Demonstration with ROS 2 nodes

First run the pybullet simulation
```
source ~/colcon_extra_ws/install/setup.sh; colcon venv sync;
cd src/res_mapf/res_mapf/packages/res_pybullet/
uv run sim --coords "0,0 2,0"
```

Next run the ROS 2 plan server
```
source install/setup.sh; source install/activate.sh;
ros2 run res_ros2 plan_server_node
```

Then the ROS 2 plan executor
```
source install/setup.sh; source install/activate.sh;
ros2 run res_ros2 plan_executor_node
```

```
ros2 topic pub -1 /robot_onboard res_ros2_msgs/RobotOnboard "robot_id: 'agent_0'
start_location: '0,0'"

ros2 topic pub -1 /robot_onboard res_ros2_msgs/RobotOnboard "robot_id: 'agent_1'
start_location: '2,0'"
```

Send initial tasks together
```
ros2 topic pub -1 /agent_0/task_request res_ros2_msgs/TaskRequest "task_id: 'agent_0_task'
robot_id: 'agent_0'
goal: '2,0'" &

ros2 topic pub -1 /agent_1/task_request res_ros2_msgs/TaskRequest "task_id: 'agent_1_task'
robot_id: 'agent_1'
goal: '0,0'"
```

Replace initial tasks with new ones
```
ros2 topic pub -1 /agent_0/task_request res_ros2_msgs/TaskRequest "task_id: 'agent_0_task'
robot_id: 'agent_0'
goal: '1,2'" &

ros2 topic pub -1 /agent_1/task_request res_ros2_msgs/TaskRequest "task_id: 'agent_1_task'
robot_id: 'agent_1'
goal: '0,3'"
```


#### Local demonstration

First run the pybullet simulation
```
source ~/colcon_extra_ws/install/setup.sh; colcon venv sync;
cd src/mapf_execution/packages/res_pybullet
uv run sim --coords "0,0 2,0"
```

Next run
```
source install/setup.sh; source install/activate.sh;
python3 src/res_ros2_ws/res_ros2/res_ros2/test/integration.py
```

