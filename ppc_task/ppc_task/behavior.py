#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from custom_interfaces.srv import GlobalPlanner
from custom_interfaces.msg import MissionMsg 

from std_msgs.msg import String
from custom_interfaces.msg import PathMsg

from custom_interfaces.action import Navigate



class BehaviorNode(Node):

    def __init__(self):
        super().__init__("behavior")
        self.current_state = "idle"

        #current state publisher
        self.state_pub = self.create_publisher(String, "/state", 10)
        self.StatePublish()
        
        #publisher to /global_plan
        self.global_plan_pub = self.create_publisher(PathMsg, '/global_plan', 10)
        
        
        #subscriber to mission to get the pose and mission_name
        self.mission_sub = self.create_subscription(MissionMsg, '/mission', self.mission_callback, 10)

        #client to /create_plan
        self.plan_client = self.create_client(GlobalPlanner, '/create_plan')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /create_plan service...")

        #client to /navigate action
        self.navigate_client = ActionClient(self, Navigate, '/navigate')

       
    def StatePublish(self):
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        self.get_logger().info('Publishing current state')


    def request_create_plan(self, target_pose):
        request = GlobalPlanner.Request()
        request.goal_pose = target_pose

        # Send the request asynchronously
        request_future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, request_future)

        # Check if the future is successful
        if request_future.result() is not None:
            self.get_logger().info(f"/create_plan client returned: {request_future.result().path}")
            return request_future.result()
        else:
            self.get_logger().error("Service(/create_plan) call failed")
            return None

    def send_goal(self, path):
        #set the goal
        self.navigate_client.wait_for_server()
        goal= Navigate.Goal()
        goal.path = path
        self.get_logger().info("Sending goal to /navigate...")

        #send the goal asynchronously with feedbackcallback
        self.goal_future= self.navigate_client.send_goal_async(goal, feedback_callback= self.feedback_callback)
        self.goal_future.add_done_callback(self.goal_response_callback)



    def goal_response_callback(self, goal_future ):
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        self.get_logger().info("Goal accepted")

        # Get result asynchronously
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)
        

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Navigation completed successfully!")
        else:
            self.get_logger().error("Navigation failed.")
        


    def feedback_callback(self, feedback_msg):
        remaining_distance = feedback_msg.feedback.remaining_distance
        self.get_logger().info(f"Remaining Distance: {remaining_distance}")


    def mission_callback(self, msg):
        self.mission_name = msg.mission_name
        self.target_pose = msg.target_pose

        if self.mission_name == "Stop":
            self.current_state = "idle"
            self.get_logger().info("Received 'Stop' mission. Returning to idle state.")
            self.StatePublish()

        elif self.mission_name == "GoTo":
            #set state to create_path and publish the state
            self.current_state = "create_path"
            self.get_logger().info("Received 'GoTo' mission. Calling /create_plan service.")
            self.StatePublish()

            # Request a path from the Global Planner and publish response to /global_plan
            response = self.request_create_plan(self.target_pose)
            if response:
                self.get_logger().info("Received path from /create_plan")
                
                path_msg = PathMsg()
                path_msg.poses = response.path 
                self.global_plan_pub.publish(path_msg) 
                self.get_logger().info("publishing to /global_plan")
                #transition to navigate state
                self.current_state = "navigate"
                self.StatePublish()

                #trigger /navigate action
                self.send_goal(response.path)

    
def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
