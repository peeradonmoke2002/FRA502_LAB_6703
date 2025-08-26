#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Int64
from turtlesim.srv import Kill

class KillerNode(Node):
    def __init__(self):
        super().__init__('killer_node')

        self.cmd_pub_2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.create_subscription(Int64,'/turtle1/pizza_count', self.pizza_count_cb, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb_1, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.pose_cb_2, 10)

        self.create_subscription(Int64, '/set_max_pizza', self.set_max_pizza_cb, 10)

        self.current_pose_turtle1 = Pose()
        self.current_pose_turtle2 = Pose()
        self.pizza_count = Int64()
        self.max_pizza_count = 20

        self.create_timer(0.05, self.on_timer)
        
        self.remove_turtle_client = self.create_client(Kill, '/remove_turtle')
        while not self.remove_turtle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for eat service...')
        
        
    def set_max_pizza_cb(self, msg):
        self.max_pizza_count = msg.data
        self.get_logger().info(f'Set max pizza to {self.max_pizza_count}')
        
        
    def eat_turtle(self,name):
        kill_req = Kill.Request()
        kill_req.name = name
        self.remove_turtle_client.call_async(kill_req)
        
        
    def pizza_count_cb(self,msg):
        self.pizza_count = msg
        
    def pose_cb_1(self, msg):
        self.current_pose_turtle1 = msg
        
    def pose_cb_2(self, msg):
        self.current_pose_turtle2 = msg
        
        
    def on_timer(self):
        if self.pizza_count.data < self.max_pizza_count:
            return

        target = self.current_pose_turtle1
        dx = target.x - self.current_pose_turtle2.x
        dy = target.y - self.current_pose_turtle2.y
        dist = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_err = angle_to_target - self.current_pose_turtle2.theta
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))

        # Gains and limits
        k_lin = 1.5
        k_ang = 4.0
        max_lin = 2.0
        max_ang = 2.0

        cmd = Twist()
        if dist < 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.eat_turtle("turtle1")
            self.cmd_pub_2.publish(cmd)

        else:
            cmd.linear.x = min(k_lin * dist, max_lin)
            cmd.angular.z = max(-max_ang, min(k_ang * angle_err, max_ang))
            self.cmd_pub_2.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = KillerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
