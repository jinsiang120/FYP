#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped ,PoseWithCovarianceStamped, Point
from builtin_interfaces.msg import Time

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float32,String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

import numpy as np 
import math
import heapq
import sys


class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

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

        self.rend_x = 5.0
        self.rend_y = -1.0
        self.rend_z = 0.0

        self.p_1_x = 0.0
        self.p_1_y = 0.0
        self.p_1_z = 0.0
        self.p_1_r_x = 0.0
        self.p_1_r_y = 0.0
        self.p_1_r_z = 0.0
        self.p_1_r_w = 0.0

        self.p_2_x = 0.0
        self.p_2_y = 0.0
        self.p_2_z = 0.0
        self.p_2_r_x = 0.0
        self.p_2_r_y = 0.0
        self.p_2_r_z = 0.0
        self.p_2_r_w = 0.0

        self.p_3_x = 0.0
        self.p_3_y = 0.0
        self.p_3_z = 0.0
        self.p_3_r_x = 0.0
        self.p_3_r_y = 0.0
        self.p_3_r_z = 0.0
        self.p_3_r_w = 0.0

        self.leader = None

        self.counter = 1
        self.counter2 = 1
        self.counter3 = 1
        self.rendezvous = 0

        
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

        self.get_logger().info("Start Rendezvous Task")



    def following(self):
     
     if self.counter2 ==1:
        self.get_logger().info("Start Herding Process")
        self.counter2 = 2

     if self.leader == 'beta_1':
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = self.rend_x
            goal.pose.position.y = self.rend_y
            goal.pose.position.z = self.rend_z
            goal.pose.orientation.w = 1.0
            self.pub1.publish(goal)

            pos_1 = PoseStamped()
            pos_1.header.frame_id = "map"
            pos_1.header.stamp = self.get_clock().now().to_msg()
            pos_1.pose.position.x = self.p_1_x 
            pos_1.pose.position.y = self.p_1_y 
            pos_1.pose.position.z = self.p_1_z 
            pos_1.pose.orientation.x = self.p_1_r_x
            pos_1.pose.orientation.y = self.p_1_r_y
            pos_1.pose.orientation.z = self.p_1_r_z
            pos_1.pose.orientation.w = self.p_1_r_w

            pos_2 = PoseStamped()
            pos_2.header.frame_id = "map"
            pos_2.header.stamp = self.get_clock().now().to_msg()
            pos_2.pose.position.x = self.p_2_x 
            pos_2.pose.position.y = self.p_2_y 
            pos_2.pose.position.z = self.p_2_z 
            pos_2.pose.orientation.x = self.p_2_r_x
            pos_2.pose.orientation.y = self.p_2_r_y
            pos_2.pose.orientation.z = self.p_2_r_z
            pos_2.pose.orientation.w = self.p_2_r_w

            pos_3 = PoseStamped()
            pos_3.header.frame_id = "map"
            pos_3.header.stamp = self.get_clock().now().to_msg()
            pos_3.pose.position.x = self.p_3_x 
            pos_3.pose.position.y = self.p_3_y 
            pos_3.pose.position.z = self.p_3_z 
            pos_3.pose.orientation.x = self.p_3_r_x
            pos_3.pose.orientation.y = self.p_3_r_y
            pos_3.pose.orientation.z = self.p_3_r_z
            pos_3.pose.orientation.w = self.p_3_r_w


            self.pub2.publish(pos_1)
            self.pub3.publish(pos_2)
            self.pub4.publish(pos_3)
    
            self.distance_goal = math.sqrt((self.rend_x -self.beta_1_x)**2 + (self.rend_y - self.beta_1_y)**2 )
            
            if self.distance_goal < 0.5 and self.counter3==1:
                self.get_logger().info("Herding Finished")
                self.counter3 = 2
                


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

        t = TransformStamped()
    
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'Position_1'

        t.transform.translation.x = self.beta_1_x - 1.0
        t.transform.translation.y = self.beta_1_y 
        t.transform.translation.z = self.beta_1_z 
        t.transform.rotation.x = self.beta_1_r_x
        t.transform.rotation.y = self.beta_1_r_y
        t.transform.rotation.z = self.beta_1_r_z
        t.transform.rotation.w = self.beta_1_r_w

        self.tf_static_broadcaster.sendTransform(t)

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'Position_2'

        t2.transform.translation.x = self.beta_1_x - 1.0
        t2.transform.translation.y = self.beta_1_y + 1.0
        t2.transform.translation.z = self.beta_1_z 
        t2.transform.rotation.x = self.beta_1_r_x
        t2.transform.rotation.y = self.beta_1_r_y
        t2.transform.rotation.z = self.beta_1_r_z
        t2.transform.rotation.w = self.beta_1_r_w


        self.tf_static_broadcaster.sendTransform(t2)

        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'map'
        t3.child_frame_id = 'Position_3'

        t3.transform.translation.x = self.beta_1_x - 1.0
        t3.transform.translation.y = self.beta_1_y - 1.0
        t3.transform.translation.z = self.beta_1_z 
        t3.transform.rotation.x = self.beta_1_r_x
        t3.transform.rotation.y = self.beta_1_r_y
        t3.transform.rotation.z = self.beta_1_r_z
        t3.transform.rotation.w = self.beta_1_r_w

        self.tf_static_broadcaster.sendTransform(t3)


        transformations_2 = [
            ('Position_1', 'map'),
            ('Position_2', 'map'),
            ('Position_3', 'map')
        ]

        transforms_2 = []

        for from_frame, to_frame in transformations_2:
            try:
                transform = self.tf_buffer.lookup_transform(
                    to_frame,
                    from_frame,
                    rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame} to {from_frame}: {ex}')
                return
                
            transforms_2.append(transform)
        
        p1,p2,p3 = transforms_2

        self.p_1_x  = p1.transform._translation.x
        self.p_1_y  = p1.transform._translation.y
        self.p_1_z  = p1.transform._translation.z
        self.p_1_r_x = p1.transform._rotation.x
        self.p_1_r_y = p1.transform._rotation.y
        self.p_1_r_z = p1.transform._rotation.z
        self.p_1_r_w = p1.transform._rotation.w

        self.p_2_x  = p2.transform._translation.x
        self.p_2_y  = p2.transform._translation.y
        self.p_2_z  = p2.transform._translation.z
        self.p_2_r_x = p2.transform._rotation.x
        self.p_2_r_y = p2.transform._rotation.y
        self.p_2_r_z = p2.transform._rotation.z
        self.p_2_r_w = p2.transform._rotation.w

        self.p_3_x  = p3.transform._translation.x
        self.p_3_y  = p3.transform._translation.y
        self.p_3_z  = p3.transform._translation.z
        self.p_3_r_x = p3.transform._rotation.x
        self.p_3_r_y = p3.transform._rotation.y
        self.p_3_r_z = p3.transform._rotation.z
        self.p_3_r_w = p3.transform._rotation.w



    def rendezvous_leader(self):

        self.distance_12 = math.sqrt((self.beta_2_x -self.beta_1_x)**2 + (self.beta_2_y - self.beta_1_y)**2 )
        self.distance_13 = math.sqrt((self.beta_3_x -self.beta_1_x)**2 + (self.beta_3_y - self.beta_1_y)**2 )
        self.distance_14 = math.sqrt((self.beta_4_x -self.beta_1_x)**2 + (self.beta_4_y - self.beta_1_y)**2 )
        self.distance_23 = math.sqrt((self.beta_3_x -self.beta_3_x)**2 + (self.beta_2_y - self.beta_3_y)**2 )  
        self.distance_24 = math.sqrt((self.beta_4_x -self.beta_2_x)**2 + (self.beta_2_y - self.beta_4_y)**2 ) 
        self.distance_34 = math.sqrt((self.beta_4_x -self.beta_3_x)**2 + (self.beta_4_y - self.beta_3_y)**2 )        


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


        if self.distance_12 < 2.0 or self.rendezvous ==1 :
            if self.distance_13 < 2.0 or self.rendezvous ==1:
                if self.distance_14 < 2.0 or self.rendezvous ==1:
                    if self.counter >3:
                        self.rendezvous = 1
                        self.following()
        if self.counter < 4:
            if self.leader == 'beta_1':
                pos_1 = PoseStamped()
                pos_1.header.frame_id = "map"
                pos_1.header.stamp = self.get_clock().now().to_msg()
                pos_1.pose.position.x = self.p_1_x 
                pos_1.pose.position.y = self.p_1_y 
                pos_1.pose.position.z = self.p_1_z 
                pos_1.pose.orientation.x = self.p_1_r_x
                pos_1.pose.orientation.y = self.p_1_r_y
                pos_1.pose.orientation.z = self.p_1_r_z
                pos_1.pose.orientation.w = self.p_1_r_w

                pos_2 = PoseStamped()
                pos_2.header.frame_id = "map"
                pos_2.header.stamp = self.get_clock().now().to_msg()
                pos_2.pose.position.x = self.p_2_x 
                pos_2.pose.position.y = self.p_2_y 
                pos_2.pose.position.z = self.p_2_z 
                pos_2.pose.orientation.x = self.p_2_r_x
                pos_2.pose.orientation.y = self.p_2_r_y
                pos_2.pose.orientation.z = self.p_2_r_z
                pos_2.pose.orientation.w = self.p_2_r_w

                pos_3 = PoseStamped()
                pos_3.header.frame_id = "map"
                pos_3.header.stamp = self.get_clock().now().to_msg()
                pos_3.pose.position.x = self.p_3_x 
                pos_3.pose.position.y = self.p_3_y 
                pos_3.pose.position.z = self.p_3_z 
                pos_3.pose.orientation.x = self.p_3_r_x
                pos_3.pose.orientation.y = self.p_3_r_y
                pos_3.pose.orientation.z = self.p_3_r_z
                pos_3.pose.orientation.w = self.p_3_r_w

                self.pub2.publish(pos_1)
                self.pub3.publish(pos_2)
                self.pub4.publish(pos_3)

                if self.counter == 1:
                    self.get_logger().info(f"Selected leader: {self.leader}")

                self.counter = 1 + self.counter 
        
        
        if self.distance_12 < 0.5:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_2_x
                goal.pose.position.y = self.beta_2_y
                goal.pose.position.z = 1.57
                goal.pose.orientation.w = 0.0
                self.pub2.publish(goal)

        if self.distance_13 < 0.5:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_3_x
                goal.pose.position.y = self.beta_3_y
                goal.pose.position.z = 3.14
                goal.pose.orientation.w = 0.0
                self.pub3.publish(goal)
        
        if self.distance_14 < 0.5:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.beta_4_x
                goal.pose.position.y = self.beta_4_y
                goal.pose.position.z = 4.517
                goal.pose.orientation.w = 0.0
                self.pub4.publish(goal)


        # Objective function
        d2 = math.sqrt((self.beta_1_x -self.p_1_x)**2 + (self.beta_1_y - self.p_1_y)**2 )
        d3 = math.sqrt((self.beta_1_x -self.p_2_x)**2 + (self.beta_1_y - self.p_2_y)**2 )
        d4 = math.sqrt((self.beta_1_x -self.p_3_x)**2 + (self.beta_1_y - self.p_3_y)**2 )

        msg = Float32()
        msg.data = (abs(math.sqrt((self.beta_1_x -self.rend_x)**2 + (self.beta_1_y - self.rend_y)**2 )) + 
                  (abs(math.sqrt((self.beta_2_x -self.beta_1_x)**2 + (self.beta_2_y - self.beta_1_y)**2 )) - d2)
                    + (abs(math.sqrt((self.beta_3_x -self.beta_1_x)**2 + (self.beta_3_y - self.beta_1_y)**2 )) -d3)
                    +((abs(math.sqrt((self.beta_4_x -self.beta_1_x)**2 + (self.beta_4_y - self.beta_1_y)**2 )))-d4))
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