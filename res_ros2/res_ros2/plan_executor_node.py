# Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
# Advanced Remanufacturing and Technology Centre
# A*STAR Research Entities (Co. Registration No. 199702110H)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.node import Node
from res_plan_execution.plan_execution.dependency_manager import DependencyManager
from res_plan_execution.plan_execution.plan_executor import PlanExecutor
from res_plan_execution.robot_controllers.shared_memory_agent_controller.agent_shmd_controller import (
    SharedMemoryAgentController,
)
from res_ros2.ros2_plan_executor_transport import Ros2PlanExecutorTransport


class PlanExecutorNode(Node):
    def __init__(self):
        super().__init__("plan_executor")

        transport = Ros2PlanExecutorTransport(self)
        robot_controller = SharedMemoryAgentController()
        dependency_manager = DependencyManager()

        self.plan_executor = PlanExecutor(
            transport=transport,
            robot_controller=robot_controller,
            dependency_manager=dependency_manager,
        )

        self.plan_executor.start()

    def destroy_node(self):
        self.plan_executor.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlanExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
