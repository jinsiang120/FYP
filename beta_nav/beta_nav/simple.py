#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped ,PoseWithCovarianceStamped, Point
from builtin_interfaces.msg import Time

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav_msgs.msg import Path

import numpy as np 
import math


class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')

        self.compute_path_to_pose_client_1 = ActionClient(self,ComputePathToPose,'/beta_1/compute_path_to_pose')
        self.compute_path_to_pose_client_2 = ActionClient(self,ComputePathToPose,'/beta_2/compute_path_to_pose')
        self.compute_path_to_pose_client_3 = ActionClient(self,ComputePathToPose,'/beta_3/compute_path_to_pose')
        self.compute_path_to_pose_client_4 = ActionClient(self,ComputePathToPose,'/beta_4/compute_path_to_pose')
        self.status = None
 
        self.timer_ =self.create_timer(1.0,self.goal)
        self.get_logger().info("GO2")
    

    def goal(self):
        start = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = 4.0
        start.pose.position.y = 1.0
        start.pose.position.z = 0.0
        start.pose.orientation.w = 1.0
    
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = -4.0
        goal.pose.position.y = -1.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

    

        self.find_path(goal,start)
        self.get_logger().info(f"Done")

    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.path.poses))

    
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)



    def get_result_callback(self, future):
        result = future.result().result
        x = result.path.poses
        a = len(x)
        b = x[10]
        goal_pose1 = PoseStamped()
        goal_pose1.header.frame_id = 'map'
        goal_pose1.header.stamp = self.get_clock().now().to_msg()
        goal_pose1.pose.position.x = b.pose.position.x 
        goal_pose1.pose.position.y = b.pose.position.x 
        goal_pose1.pose.orientation.w = b.pose.position.x 
        goal_pose1.pose.orientation.z = b.pose.orientation.x 
        self.get_logger().info(f"Path Length:{a}")
        self.get_logger().info(f"Goal of x:{goal_pose1.pose.position.x}")
        rclpy.shutdown()



    def find_path(self,goal,start):

        planner_id = ''
        use_start = False

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = planner_id
        goal_msg.use_start = use_start

       

        self.compute_path_to_pose_client_1.wait_for_server()


        self._send_goal_future = self.compute_path_to_pose_client_1.send_goal_async(
        goal_msg,
        feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
      
 


def main(args=None):
    rclpy.init(args=args)

    goal_publisher = GoalPublisher()

    rclpy.spin(goal_publisher)

    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()