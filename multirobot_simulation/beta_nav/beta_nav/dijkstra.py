#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped ,PoseWithCovarianceStamped, Point
from builtin_interfaces.msg import Time

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
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

        self.rend_x = 5.0
        self.rend_y = -1.5
        self.rend_z = 0.0

        self.leader = None

        self.counter = 1
        self.rendezvous = 0
        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
             'target_frame', 'beta_1/base_link').get_parameter_value().string_value
        
        # Create a publisher on the "goal_pose" topic
        self.pub1 = self.create_publisher(PoseStamped, 'beta_1/goal_pose',10 )
        self.pub2 = self.create_publisher(PoseStamped, 'beta_2/goal_pose',10 )
        self.pub3 = self.create_publisher(PoseStamped, 'beta_3/goal_pose',10 )
        self.pub4 = self.create_publisher(PoseStamped, 'beta_4/goal_pose',10 )

        self.pub5 = self.create_publisher(Float32,'ObjFunc',10)

        self.pub6 = self.create_publisher(Float32,'Beta_1',10)
        self.pub7 = self.create_publisher(Float32,'Beta_2',10)
        self.pub8 = self.create_publisher(Float32,'Beta_3',10)
        self.pub9 = self.create_publisher(Float32,'Beta_4',10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        
        self.timer_ =self.create_timer(1.0,self.position)
        self.timer_ =self.create_timer(1.0,self.rendezvous_leader)

        self.get_logger().info("GO2")



    def following(self):
     if self.leader == 'beta_1':
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = self.rend_x
            goal.pose.position.y = self.rend_y
            goal.pose.position.z = self.rend_z
            goal.pose.orientation.w = 1.0
            self.pub1.publish(goal)

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
                goal.pose.position.z = self.beta_2_z
                goal.pose.orientation.w = 0.0
                self.pub2.publish(goal)

            if self.distance_13 < 1.5:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_3_x
                goal.pose.position.y = self.beta_3_y
                goal.pose.position.z = self.beta_3_z
                goal.pose.orientation.w = 0.0
                self.pub3.publish(goal)
        
            if self.distance_14 < 1.5:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_4_x
                goal.pose.position.y = self.beta_4_y
                goal.pose.position.z = self.beta_4_z
                goal.pose.orientation.w = 0.0
                self.pub4.publish(goal)

            if self.distance_12 < 1.5 and self.distance_13 < 1.5 and self.distance_14 < 1.5:
                self.rendezvous = 1
                self.get_logger().info("Rendezvous")


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


        self.beta_2_x = t2.transform._translation.x 
        self.beta_2_y = t2.transform._translation.y
        self.beta_2_z = t2.transform._translation.z

        self.beta_3_x = t3.transform._translation.x 
        self.beta_3_y = t3.transform._translation.y
        self.beta_3_z = t3.transform._translation.z 


        self.beta_4_x = t4.transform._translation.x
        self.beta_4_y = t4.transform._translation.y
        self.beta_4_z = t4.transform._translation.z

        x = (t1.transform._translation.x + t2.transform._translation.x + t3.transform._translation.x + t4.transform._translation.x)/4
        y = (t1.transform._translation.y + t2.transform._translation.y + t3.transform._translation.y + t4.transform._translation.y)/4
        z = (t1.transform._translation.z + t2.transform._translation.z + t3.transform._translation.z + t4.transform._translation.z)/4
       
 

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


        if self.distance_12 < 1.5 or self.rendezvous ==1 :
            if self.distance_13 < 1.5 or self.rendezvous ==1:
                if self.distance_14 < 1.5 or self.rendezvous ==1:
                    if self.counter >3:
                        self.rendezvous = 1
                        self.get_logger().info("Start")
                        self.following()

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
        
        if self.distance_12 < 1.0:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_2_x
                goal.pose.position.y = self.beta_2_y
                goal.pose.position.z = self.beta_2_z
                goal.pose.orientation.w = 0.0
                self.pub2.publish(goal)

        if self.distance_13 < 1.0:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_3_x
                goal.pose.position.y = self.beta_3_y
                goal.pose.position.z = self.beta_3_z
                goal.pose.orientation.w = 0.0
                self.pub3.publish(goal)
        
        if self.distance_14 < 1.0:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_4_x
                goal.pose.position.y = self.beta_4_y
                goal.pose.position.z = self.beta_4_z
                goal.pose.orientation.w = 0.0
                self.pub4.publish(goal)

        # Objective Function
        msg = Float32()
        msg.data = (abs(math.sqrt((self.beta_1_x -self.rend_x)**2 + (self.beta_1_y - self.rend_y)**2 )) + 
                    abs(math.sqrt((self.beta_2_x -self.rend_x)**2 + (self.beta_2_y - self.rend_y)**2 )) 
                    + abs(math.sqrt((self.beta_3_x -self.rend_x)**2 + (self.beta_3_y - self.rend_y)**2 )) +
                     abs(math.sqrt((self.beta_4_x -self.rend_x)**2 + (self.beta_4_y - self.rend_y)**2 )))
     
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