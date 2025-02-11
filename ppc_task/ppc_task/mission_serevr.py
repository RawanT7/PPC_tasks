#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import Mission
from custom_interfaces.msg import MissionMsg 
from std_msgs.msg import String
from geometry_msgs.msg import Pose


class MissionServerNode(Node):

    def __init__(self):
        super().__init__("mission_server")
        self.server = self.create_service(Mission, "/start_mission", self.server_callback)
        self.publisher = self.create_publisher(String, '/mission', 10)
        self.get_logger().info("Mission node is initialized.")

    def server_callback(self, request, response):
        available_names = ["GoTo", "Stop"]
        
        if request.mission_name in available_names:
            response.accepted = True

            # Publish mission name
            mission_msg = MissionMsg()
            mission_msg.mission_name = request.mission_name
            mission_msg.target_pose = request.target_pose

            self.get_logger().info(f"Mission accepted.")
        else:
            response.accepted = False
            self.get_logger().info(f"Mission rejected.")

        return response  #return of response


def main(args=None):
    rclpy.init(args=args)
    node = MissionServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
