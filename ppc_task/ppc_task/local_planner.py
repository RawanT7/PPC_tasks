#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from  rclpy.action import ActionServer
from  rclpy.action.server import ServerGoalHandle
from custom_interfaces.action import Navigate
from geometry_msgs.msg import Twist
import math


class LocalPlannerNode(Node):

    def __init__(self):
        super().__init__("local_planner")
        self.navigate_server = ActionServer(self, Navigate, "local_planner", 
                                            execute_callback= self.execute_callback)
        
       # Publisher for robot velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("local planner and /navigate action are intitialized ")     #to show log info
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        #define request
        path = goal_handle.request.path 
        if len(path) < 2:
            self.get_logger().error("Invalid path received. At least two poses required.")
            goal_handle.abort()
            return Navigate.Result(success=False)

        for i in range(len(path) - 1):
            start_pose = path[i]
            target_pose = path[i + 1]

        #performing action
        if not self.rotate(start_pose, target_pose):
                self.get_logger().error("Rotation failed.")
                goal_handle.abort()
                return Navigate.Result(success=False)
            
        if not self.move_forward(target_pose):
            self.get_logger().error("Movement failed.")
            goal_handle.abort()
            return Navigate.Result(success=False)

        #set final goal state : 
        goal_handle.succeed()

        #send the result 
        result = Navigate.Result(success = True)
        return result

    def rotate(self, start, target):
        dx = target.pose.position.x - start.pose.position.x
        dy = target.pose.position.y - start.pose.position.y
        target_yaw = math.atan2(dy, dx)
        twist_msg = Twist()

        while abs(target_yaw) > 0.1:
            twist_msg.angular.z = 0.5 * target_yaw / abs(target_yaw)
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info("Rotating...")
        
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        return True

    def move_forward(self, target):
        twist_msg = Twist()
        twist_msg.linear.x = 0.2
        
        while True:  # Simplified condition, should be improved with real odometry tracking
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info("Moving forward...")
            # Assume target reached for simplicity
            break
        
        twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        return True

    
def main(args=None):
    rclpy.init(args=args)

    node = LocalPlannerNode()
    rclpy.spin(node)

    rclpy.shutdown()
    

if __name__ == '__main__':
    main()