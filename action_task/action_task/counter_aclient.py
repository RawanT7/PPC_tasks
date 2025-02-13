#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_interfaces.action import Counter

#to create the node 
class CounterAClientNode(Node):

    def __init__(self):
        super().__init__("counter_aclient")
        self.counter_aclient = ActionClient(self, Counter, "counter" )
       

    def send_goal(self, start_number, target_number):
        #wait for server to connect
        self.counter_aclient.wait_for_server()
        
        #create a goal
        goal = Counter.Goal()
        goal.start_number = int(start_number)
        goal.target_number = int(target_number)
        self.get_logger().info(f"Hello mind cloud, sending goal, start:{start_number}, target:{target_number}")     
        
        #send the goal with feedbackcallback
        future= self.counter_aclient.send_goal_async(goal, feedback_callback= self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback received - Current count: {feedback_msg.feedback.current_count}")

    #when the goal is send give response with result
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Final count: {result.count}")



def main(args=None):
    rclpy.init(args=args)

    start_number= input("enter start number")
    target_number = input("enter the target number")

    node = CounterAClientNode()

    node.send_goal(start_number, target_number)
    rclpy.spin(node)



    rclpy.shutdown()
    

if __name__ == '__main__':
    main()