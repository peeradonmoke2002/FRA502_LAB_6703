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
        # Publisher for turtle velocity
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Subscriptions
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.create_subscription(Point, '/mouse_position', self.mouse_pos_cb, 10)
        self.create_subscription(Int64,'/turtle1/pizza_count', self.pizza_count_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose, 10)

        # Internal state
        self.current_pose = Pose()
        self.target = []
        self.last_spawn = None
        self.pizza_count = Int64()

        # Control loop timer
        self.create_timer(0.05, self.on_timer)

        self.spawn_client = self.create_client(GivePosition, 'spawn_pizza')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn_pizza service...')
       
        self.eat_client = self.create_client(Empty, '/turtle1/eat')
        while not self.eat_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for eat service...')

    def pizza_count_cb(self,msg):
        self.pizza_count = msg
        # self.get_logger().info(f'Current pizza count: {self.pizza_count.data}')
        
    def eat_pizza(self):
        eat_req = Empty.Request()
        self.eat_client.call_async(eat_req)
        
    def spawn_pizza(self,x,y):
        spawn_req = GivePosition.Request()
        spawn_req.x = x
        spawn_req.y = y
        self.spawn_client.call_async(spawn_req)
        
    def mouse_pos_cb(self, msg: Point):
        """Handle new click positions; spawn pizza once per unique click."""
        pos = (round(msg.x, 2), round(msg.y, 2))
        if self.last_spawn != pos:
            if self.pizza_count.data >= 20:
                self.get_logger().info('Pizza limit reached; not spawning new pizza.')
            else:
                self.spawn_pizza(msg.x, msg.y)
            # self.get_logger().info(f"Spawned pizza at {pos}")
            if not hasattr(self, 'targets'):
                self.targets = []
            self.targets.append(msg)
            self.last_spawn = pos
            
    def goal_pose(self, msg: PoseStamped):
        """Handle new goal pose; update target if different."""
        pos = (round(msg.x, 2), round(msg.y, 2))
        if self.last_spawn != pos:
            if self.pizza_count.data >= 20:
                self.get_logger().info('Pizza limit reached; not spawning new pizza.')
            else:
                self.spawn_pizza(msg.x, msg.y)
            # self.get_logger().info(f"Spawned pizza at {pos}")
            if not hasattr(self, 'targets'):
                self.targets = []
            self.targets.append(msg)
            self.last_spawn = pos
            

    def pose_cb(self, msg: Pose):
        """Update current turtle pose."""
        self.current_pose = msg

    def on_timer(self):
        """Control loop: navigate to target and call eat when close enough."""
        if not hasattr(self, 'targets') or not self.targets:
            return

        # Go to the first target in the list
        target = self.targets[0]
        dx = target.x - self.current_pose.x
        dy = target.y - self.current_pose.y
        dist = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_err = angle_to_target - self.current_pose.theta
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))  # normalize

        # Gains and limits
        k_lin = 1.5
        k_ang = 4.0
        max_lin = 2.0
        max_ang = 2.0

        cmd = Twist()
        if dist < 0.5:
            # Arrived: stop and eat
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Arrived at pizza; eating!')
            if self.pizza_count.data < 20:
                self.eat_pizza()
            # Remove the reached target
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
