#!/usr/bin/python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtlesim_plus_interfaces.srv import GivePosition
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped

class EaterNode(Node):
    def __init__(self):
        super().__init__('eater_node')

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.create_subscription(Point, '/mouse_position', self.mouse_pos_cb, 10)
        self.create_subscription(Int64,'/turtle1/pizza_count', self.pizza_count_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose, 10)


        self.current_pose = Pose()
        self.target = []
        self.last_spawn = None
        self.pizza_count = Int64()
        self.spawn_requests_count = 0
        self.max_pizza_count = 20
        
        self.create_subscription(Int64, '/set_max_pizza', self.set_max_pizza_cb, 10)

        self.create_timer(0.05, self.on_timer)

        self.spawn_client = self.create_client(GivePosition, 'spawn_pizza')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn_pizza service...')
       
        self.eat_client = self.create_client(Empty, '/turtle1/eat')
        while not self.eat_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for eat service...')

    def pizza_count_cb(self, msg):
        self.pizza_count = msg

        if self.spawn_requests_count < msg.data:
            self.spawn_requests_count = msg.data

        
    def set_max_pizza_cb(self, msg):
        self.max_pizza_count = msg.data
        self.get_logger().info(f'Set max pizza to {self.max_pizza_count}')
        
    def eat_pizza(self):
        eat_req = Empty.Request()
        self.eat_client.call_async(eat_req)
        
    def spawn_pizza(self,x,y):

        if self.spawn_requests_count >= self.max_pizza_count:
            self.get_logger().info(f'Pizza limit ({self.max_pizza_count}) reached; not spawning new pizza.')
            return
        

        self.spawn_requests_count += 1
        
        spawn_req = GivePosition.Request()
        spawn_req.x = x
        spawn_req.y = y
        self.spawn_client.call_async(spawn_req)
        
    def mouse_pos_cb(self, msg: Point):
        pos = (round(msg.x, 2), round(msg.y, 2))
        if self.last_spawn != pos:
      
            if self.spawn_requests_count >= self.max_pizza_count:
                self.get_logger().info(f'Pizza limit ({self.max_pizza_count}) reached; not spawning new pizza.')
            else:
                self.spawn_pizza(msg.x, msg.y)
                

            if not hasattr(self, 'targets'):
                self.targets = []
            self.targets.append(msg)
            self.last_spawn = pos
            
    def goal_pose(self, msg: PoseStamped):

        self.get_logger().info(f"Received goal_pose: x={msg.pose.position.x}, y={msg.pose.position.y}")
        pos = (round(msg.pose.position.x, 2), round(msg.pose.position.y, 2))
        if self.last_spawn != pos:

            if self.spawn_requests_count >= self.max_pizza_count:
                self.get_logger().info(f'Pizza limit ({self.max_pizza_count}) reached; not spawning new pizza.')
            else:
                self.spawn_pizza(msg.pose.position.x, msg.pose.position.y)
                
            if not hasattr(self, 'targets'):
                self.targets = []

            point = Point()
            point.x = msg.pose.position.x
            point.y = msg.pose.position.y
            point.z = msg.pose.position.z
            self.targets.append(point)
            self.last_spawn = pos
            

    def pose_cb(self, msg: Pose):
        self.current_pose = msg

    def on_timer(self):
        if not hasattr(self, 'targets') or not self.targets:
            return

        target = self.targets[0]
        dx = target.x - self.current_pose.x
        dy = target.y - self.current_pose.y
        dist = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_err = angle_to_target - self.current_pose.theta
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err)) 

        k_lin = 1.5
        k_ang = 4.0
        max_lin = 2.0
        max_ang = 2.0

        cmd = Twist()
        if dist < 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
            if self.pizza_count.data <= self.max_pizza_count:
                self.get_logger().info('Arrived at pizza; eating!')
                self.eat_pizza()
            else:
                self.get_logger().info(f'Arrived at position; pizza limit ({self.max_pizza_count}) reached.')
                
            self.targets.pop(0)
            self.cmd_pub.publish(cmd)

        else:
            cmd.linear.x = min(k_lin * dist, max_lin)
            cmd.angular.z = max(-max_ang, min(k_ang * angle_err, max_ang))
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = EaterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
