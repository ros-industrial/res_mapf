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


import logging
import rclpy
from rclpy.node import Node
from res_mapf_planning.mapf_solve.solvers.cbs_adapter import CBSAdapter
from res_mapf_planning.planning.mapf_coordinator import MAPFCoordinator
from res_mapf_planning.planning.multi_agent_context import MultiAgentContext
from res_mapf_planning.traffic_dependencies.plan_generator import PlanGenerator
from res_plan_server.plan_server import PlanServer
from res_ros2.ros2_plan_server_transport import Ros2PlanServerTransport

logging.basicConfig(
    level=logging.DEBUG,
    format="[%(levelname)s] %(asctime)s %(name)s: %(message)s",
)


class PlanServerNode(Node):
    def __init__(self):
        super().__init__("plan_server")

        transport = Ros2PlanServerTransport(self)
        context = MultiAgentContext()
        solver = CBSAdapter()
        coordinator = MAPFCoordinator(context, solver)
        plan_generator = PlanGenerator()

        logger = logging.getLogger("PlanServerNode")
        logging.basicConfig(
            level=logging.INFO,
            format="[%(levelname)s] %(asctime)s %(name)s: %(message)s",
        )

        self.plan_server = PlanServer(
            transport=transport,
            context=context,
            coordinator=coordinator,
            plan_generator=plan_generator,
            logger=logger,
        )
        self.plan_server.start()

    def destroy_node(self):
        self.plan_server.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlanServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
