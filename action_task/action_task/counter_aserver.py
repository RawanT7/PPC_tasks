#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from  rclpy.action import ActionServer
from  rclpy.action.server import ServerGoalHandle
from custom_interfaces.action import Counter


class CounterAServerNode(Node):

    def __init__(self):
        super().__init__("counter_aserver")
        self.counter_aserver = ActionServer(self, Counter, "counter", 
                                            execute_callback= self.execute_callback)
        self.get_logger().info("Hello mind cloud, action server started")     #to show log info
       
    def execute_callback(self, goal_handle: ServerGoalHandle):
        #define request
        start_number = goal_handle.request.start_number 
        target_number = goal_handle.request.target_number

        #performing action
        current_count = start_number
        feedback = Counter.Feedback()

        for i in range (start_number, target_number):
            current_count += 1 
            self.get_logger().info(f"count is : {current_count}") 
            #send feedback
            feedback._current_count = current_count
            goal_handle.publish_feedback(feedback)

            time.sleep(2)
        self.get_logger().info("target is reached")

        #set final goal state : 
        goal_handle.succeed()

        #send the result 
        result = Counter.Result()
        result.count = current_count
        return result

def main(args=None):
    rclpy.init(args=args)

    node = CounterAServerNode()
    rclpy.spin(node)

    rclpy.shutdown()
    

if __name__ == '__main__':
    main()