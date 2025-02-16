#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import GlobalPlanner 
from nav_msgs.msg import Odometry
from custom_interfaces.msg import PathMsg
from geometry_msgs.msg import Pose

class GlobalPlannerNode(Node):

    def __init__(self):
        super().__init__("global_planner")
        #/create_plan service
        self.create_plan_server = self.create_service(GlobalPlanner , "/create_plan", self.create_plan_callback) 
        # subscriber to odom
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.get_logger().info("initialized /create_plan .")
    def create_plan_callback(self, request, response):
        if self.current_pose is None:
            self.get_logger().error("No current position received yet.")
            return response

        self.get_logger().info("Received /create_plan request.")
        self.path_msg = PathMsg()
        start_pose =  Pose()
        start_pose.position = self.current_pose.position
        start_pose.orientation = self.current_pose.orientation

        #reveive the goal from request
        goal_pose= request.goal_pose 

        #add current and goal poses
        self.path_msg.poses.append(start_pose)
        self.path_msg.poses.append(goal_pose)

        # Return path as response
        response.path = self.path_msg.poses
        self.get_logger().info("Generated straight-line path from current position to goal.")
        return response
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
