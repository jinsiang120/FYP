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
from std_msgs.msg import Float32,String

import numpy as np 
import math
import heapq
import sys


class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')


        self.beta_1_x = 0.0
        self.beta_1_y = 0.0
        self.beta_1_z = 0.0
        self.beta_1_r_x = 0.0
        self.beta_1_r_y = 0.0
        self.beta_1_r_z = 0.0
        self.beta_1_r_w = 0.0

        self.beta_2_x = 0.0
        self.beta_2_y = 0.0
        self.beta_2_z = 0.0

        self.beta_3_x = 0.0
        self.beta_3_y = 0.0
        self.beta_3_z = 0.0

        self.beta_4_x = 0.0
        self.beta_4_y = 0.0
        self.beta_4_z = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = 0.0

        self.rend_x = 0.0
        self.rend_y = 3.0
        self.rend_z = 0.0
        self.path_poses = []

        self.leader = None
        self.counter = 1
        self.rendezvous = 0

        self.compute_path_to_pose_client_1 = ActionClient(self,ComputePathToPose,'/beta_1/compute_path_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub1 = self.create_publisher(PoseStamped, 'beta_1/goal_pose',10 )
        self.pub2 = self.create_publisher(PoseStamped, 'beta_2/goal_pose',10 )
        self.pub3 = self.create_publisher(PoseStamped, 'beta_3/goal_pose',10 )
        self.pub4 = self.create_publisher(PoseStamped, 'beta_4/goal_pose',10 )

        self.pub5 = self.create_publisher(Float32,'ObjFunc',10)

        self.pub6 = self.create_publisher(Float32,'Beta_1',10)
        self.pub7 = self.create_publisher(Float32,'Beta_2',10)
        self.pub8 = self.create_publisher(Float32,'Beta_3',10)
        self.pub9 = self.create_publisher(Float32,'Beta_4',10)

        self.status = None
        self.init = 0 

        self.timer_ =self.create_timer(1.0,self.position)
        self.timer_ =self.create_timer(1.0,self.rendezvous_leader)

        self.get_logger().info("GO")
    

    def path_finder(self):
        start = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = self.beta_1_x
        start.pose.position.y = self.beta_1_y
        start.pose.position.z = self.beta_1_z
        start.pose.orientation.x = self.beta_1_r_x 
        start.pose.orientation.y = self.beta_1_r_y 
        start.pose.orientation.z = self.beta_1_r_z 
        start.pose.orientation.w = self.beta_1_r_w 
    
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = -4.0
        goal.pose.position.y = -1.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

    
        # Find the Final Rendezvous Location
        self.find_path(goal,start)
        self.get_logger().info(f"Done")


#######################################################################################
    def set_path(self):

        l = len(self.path_poses)
        x = self.path_poses

        goal_poses = []
        if (l >= 25) and (l<50):

            b = x[25]
            goal_pose1 = PoseStamped()
            goal_pose1.header.frame_id = 'map'
            goal_pose1.header.stamp = self.get_clock().now().to_msg()
            goal_pose1.pose.position.x = b.pose.position.x 
            goal_pose1.pose.position.y = b.pose.position.y
            goal_pose1.pose.position.z = b.pose.position.z  
            goal_pose1.pose.orientation.w = b.pose.orientation.w
            goal_pose1.pose.orientation.z = b.pose.orientation.x 
            goal_pose1.pose.orientation.y = b.pose.orientation.y 
            goal_pose1.pose.orientation.z = b.pose.orientation.z 
            goal_poses.append(goal_pose1)

        elif (l >= 50) and (l<75):

            b = x[25]
            goal_pose1 = PoseStamped()
            goal_pose1.header.frame_id = 'map'
            goal_pose1.header.stamp = self.get_clock().now().to_msg()
            goal_pose1.pose.position.x = b.pose.position.x 
            goal_pose1.pose.position.y = b.pose.position.y
            goal_pose1.pose.position.z = b.pose.position.z  
            goal_pose1.pose.orientation.w = b.pose.orientation.w
            goal_pose1.pose.orientation.z = b.pose.orientation.x 
            goal_pose1.pose.orientation.y = b.pose.orientation.y 
            goal_pose1.pose.orientation.z = b.pose.orientation.z
            goal_poses.append(goal_pose1)

            c = x[50]
            goal_pose2 = PoseStamped()
            goal_pose2.header.frame_id = 'map'
            goal_pose2.header.stamp = self.get_clock().now().to_msg()
            goal_pose2.pose.position.x = c.pose.position.x 
            goal_pose2.pose.position.y = c.pose.position.y
            goal_pose2.pose.position.z = c.pose.position.z  
            goal_pose2.pose.orientation.w = c.pose.orientation.w
            goal_pose2.pose.orientation.z = c.pose.orientation.x 
            goal_pose2.pose.orientation.y = c.pose.orientation.y 
            goal_pose2.pose.orientation.z = c.pose.orientation.y
            goal_poses.append(goal_pose2)

        elif (l> 75):

            b = x[25]
            goal_pose1 = PoseStamped()
            goal_pose1.header.frame_id = 'map'
            goal_pose1.header.stamp = self.get_clock().now().to_msg()
            goal_pose1.pose.position.x = b.pose.position.x 
            goal_pose1.pose.position.y = b.pose.position.y
            goal_pose1.pose.position.z = b.pose.position.z  
            goal_pose1.pose.orientation.w = b.pose.orientation.w
            goal_pose1.pose.orientation.z = b.pose.orientation.x 
            goal_pose1.pose.orientation.y = b.pose.orientation.y 
            goal_pose1.pose.orientation.z = b.pose.orientation.z
            goal_poses.append(goal_pose1)

            c = x[50]
            goal_pose2 = PoseStamped()
            goal_pose2.header.frame_id = 'map'
            goal_pose2.header.stamp = self.get_clock().now().to_msg()
            goal_pose2.pose.position.x = c.pose.position.x 
            goal_pose2.pose.position.y = c.pose.position.y
            goal_pose2.pose.position.z = c.pose.position.z  
            goal_pose2.pose.orientation.w = c.pose.orientation.w
            goal_pose2.pose.orientation.z = c.pose.orientation.x 
            goal_pose2.pose.orientation.y = c.pose.orientation.y 
            goal_pose2.pose.orientation.z = c.pose.orientation.y
            goal_poses.append(goal_pose2)

            d = x[75]
            goal_pose3 = PoseStamped()
            goal_pose3.header.frame_id = 'map'
            goal_pose3.header.stamp = self.get_clock().now().to_msg()
            goal_pose3.pose.position.x = d.pose.position.x 
            goal_pose3.pose.position.y = d.pose.position.y
            goal_pose3.pose.position.z = d.pose.position.z  
            goal_pose3.pose.orientation.w = d.pose.orientation.w
            goal_pose3.pose.orientation.z = d.pose.orientation.x 
            goal_pose3.pose.orientation.y = d.pose.orientation.y 
            goal_pose3.pose.orientation.z = d.pose.orientation.y
            goal_poses.append(goal_pose3)

        if l != 0:
            a = x[l-1]
            final_pose = PoseStamped()
            final_pose.header.frame_id = 'map'
            final_pose.header.stamp = self.get_clock().now().to_msg()
            final_pose.pose.position.x = a.pose.position.x 
            final_pose.pose.position.y = a.pose.position.y
            final_pose.pose.position.z = a.pose.position.z  
            final_pose.pose.orientation.w = a.pose.orientation.w
            final_pose.pose.orientation.z = a.pose.orientation.x 
            final_pose.pose.orientation.y = a.pose.orientation.y 
            final_pose.pose.orientation.z = a.pose.orientation.y
            goal_poses.append(final_pose)

        return(goal_poses)

 #######################################################################################
    
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
        self.path_poses = result.path.poses
        self.get_logger().info(f"Done Find Path")
     

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

#####################################################################################3
# Function to update the location of each robot

    def position(self):
        transformations = [
            ('beta_1/base_link', 'beta_1/odom'),
            ('beta_2/base_link', 'beta_2/odom'),
            ('beta_3/base_link', 'beta_3/odom'),
            ('beta_4/base_link', 'beta_4/odom')
        ]

        transforms = []

        for from_frame, to_frame in transformations:
            try:
                transform = self.tf_buffer.lookup_transform(
                    to_frame,
                    from_frame,
                    rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame} to {from_frame}: {ex}')
                return
                
            transforms.append(transform)

        t1, t2, t3, t4 = transforms

        self.beta_1_x = t1.transform._translation.x
        self.beta_1_y = t1.transform._translation.y
        self.beta_1_z = t1.transform._translation.z
        self.beta_1_r_x = t1.transform._rotation.x
        self.beta_1_r_y = t1.transform._rotation.y
        self.beta_1_r_z = t1.transform._rotation.z
        self.beta_1_r_w = t1.transform._rotation.w
        
        self.beta_2_x = t2.transform._translation.x 
        self.beta_2_y = t2.transform._translation.y
        self.beta_2_z = t2.transform._translation.z

        self.beta_3_x = t3.transform._translation.x 
        self.beta_3_y = t3.transform._translation.y
        self.beta_3_z = t3.transform._translation.z 

        self.beta_4_x = t4.transform._translation.x
        self.beta_4_y = t4.transform._translation.y
        self.beta_4_z = t4.transform._translation.z


# Function to elect a leader and rendezvous towards the leader

    def rendezvous_leader(self):

        self.distance_12 = math.sqrt((self.beta_2_x -self.beta_1_x)**2 + (self.beta_2_y - self.beta_1_y)**2 )
        self.distance_13 = math.sqrt((self.beta_3_x -self.beta_1_x)**2 + (self.beta_3_y - self.beta_1_y)**2 )
        self.distance_14 = math.sqrt((self.beta_4_x -self.beta_1_x)**2 + (self.beta_4_y - self.beta_1_y)**2 )

        self.distance_23 = math.sqrt((self.beta_3_x -self.beta_3_x)**2 + (self.beta_2_y - self.beta_2_y)**2 )  
        self.distance_24 = math.sqrt((self.beta_4_x -self.beta_1_x)**2 + (self.beta_2_y - self.beta_2_y)**2 ) 

        self.distance_34 = math.sqrt((self.beta_4_x -self.beta_1_x)**2 + (self.beta_3_y - self.beta_3_y)**2 )        


        nodes = ['beta_1', 'beta_2', 'beta_3', 'beta_4']
        distances = {}


        for start_node in nodes:
        # Graph Representation
            graph = {
            'beta_1':{'beta_2':self.distance_12,'beta_3':self.distance_13,'beta_4':self.distance_14},
            'beta_2':{'beta_1':self.distance_12,'beta_3':self.distance_23,'beta_4':self.distance_24},
            'beta_3':{'beta_1':self.distance_13,'beta_2':self.distance_23,'beta_4':self.distance_34},
            'beta_4':{'beta_1':self.distance_14,'beta_2':self.distance_24,'beta_3':self.distance_34},
            }
            distances[start_node] = dijkstra(graph, start_node)

        leader_distances = {}

        for node, distance in distances.items():
             leader_distances[node] = distance['beta_1'] # distance from node to beta_1
        
        self.leader = select_leader(leader_distances)


        # If the Rendezvous of Leader is finished, start to go to the final location
        if self.distance_12 < 2 or self.rendezvous ==1 :
            if self.distance_13 < 2 or self.rendezvous ==1:
                if self.distance_14 < 2 or self.rendezvous ==1:
                    if self.counter >3:
                        self.rendezvous = 1
                        self.get_logger().info("Start")
                        self.final_location()

        else:
            self.get_logger().info(f"Distance between beta_1 and beta_2: {self.distance_12}")
            self.get_logger().info(f"Distance between beta_1 and beta_3: {self.distance_13}")
            self.get_logger().info(f"Distance between beta_1 and beta_4: {self.distance_14}")

        if self.counter < 4:
            if self.leader == 'beta_1':
                self.get_logger().info(f"Selected leader: {self.leader}")
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_1_x
                goal.pose.position.y = self.beta_1_y
                goal.pose.position.z = self.beta_1_z
                goal.pose.orientation.w = 1.0
                self.pub2.publish(goal)
                self.pub3.publish(goal)
                self.pub4.publish(goal)
                self.get_logger().info(f"Counter:= {self.counter}")
                self.counter = self.counter + 1
                self.get_logger().info(f"Goal: x={goal.pose.position.x }, y={goal.pose.position.y}, z={goal.pose.position.y}")
        
        if self.distance_12 < 1.5:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_2_x
                goal.pose.position.y = self.beta_2_y
                goal.pose.position.z = 1.57
                goal.pose.orientation.w = 0.0
                self.pub2.publish(goal)

        if self.distance_13 < 1.5:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_3_x
                goal.pose.position.y = self.beta_3_y
                goal.pose.position.z = 3.14
                goal.pose.orientation.w = 0.0
                self.pub3.publish(goal)
        
        if self.distance_14 < 1.5:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_4_x
                goal.pose.position.y = self.beta_4_y
                goal.pose.position.z = 4.517
                goal.pose.orientation.w = 0.0
                self.pub4.publish(goal)

        
        # Objective Function
        msg = Float32()
        msg.data = self.beta_1_x - self.rend_x
        msg.data = self.beta_2_x - self.rend_x + abs(msg.data)
        msg.data = self.beta_3_x - self.rend_x + abs(msg.data)
        msg.data = self.beta_4_x - self.rend_x + abs(msg.data)
        msg.data = abs(msg.data)
        self.pub5.publish(msg)
    
        msg2 = Float32()
        msg2.data = self.beta_1_x
        self.pub6.publish(msg2)
        msg2.data = self.beta_2_x
        self.pub7.publish(msg2)
        msg2.data = self.beta_3_x
        self.pub8.publish(msg2)
        msg2.data = self.beta_4_x
        self.pub9.publish(msg2)


    def final_location(self):
     
     goal_poses = []
     self.path_finder()

     goal_poses = self.set_path()

     if (self.leader == 'beta_1') and (goal_poses != []):
            self.pub1.publish(goal_poses)

            leader_pos = PoseStamped()
            leader_pos.header.frame_id = "map"
            leader_pos.header.stamp = self.get_clock().now().to_msg()
            leader_pos.pose.position.x = self.beta_1_x
            leader_pos.pose.position.y = self.beta_1_y
            leader_pos.pose.position.z = self.beta_1_z
            leader_pos.pose.orientation.w = 1.0
            self.pub2.publish(leader_pos)
            self.pub3.publish(leader_pos)
            self.pub4.publish(leader_pos)
    
            if self.distance_12 < 1.5:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_2_x
                goal.pose.position.y = self.beta_2_y
                goal.pose.position.z = 1.57
                goal.pose.orientation.w = 0.0
                self.pub2.publish(goal)

            if self.distance_13 < 1.5:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_3_x
                goal.pose.position.y = self.beta_3_y
                goal.pose.position.z = self.beta_3_z
                goal.pose.orientation.w = 3.14
                self.pub3.publish(goal)
        
            if self.distance_14 < 1.5:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_4_x
                goal.pose.position.y = self.beta_4_y
                goal.pose.position.z = 4.712
                goal.pose.orientation.w = 0.0
                self.pub4.publish(goal)

            if self.distance_12 < 1.5 and self.distance_13 < 1.5 and self.distance_14 < 1.5:
                self.rendezvous = 1
                self.get_logger().info("Rendezvous")


def dijkstra(graph,start_node):

    distances = {node: sys.maxsize for node in graph}
    distances[start_node] = 0
    heap = [(0, start_node)]
    visited = set()

    while heap:
        (current_distance, current_node) = heapq.heappop(heap)

        if current_node in visited:
            continue

        visited.add(current_node)

        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(heap, (distance, neighbor))

    return distances


def select_leader(distances):
    shortest_distance = sys.maxsize
    for node, distance in distances.items():
        if distance < shortest_distance:
            shortest_distance = distance
            leader = node
    return leader


 

def main(args=None):
    rclpy.init(args=args)

    goal_publisher = GoalPublisher()

    rclpy.spin(goal_publisher)

    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()