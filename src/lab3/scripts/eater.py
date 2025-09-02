#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from controller_interfaces.srv import SetMaxPizza, SetParam
from std_srvs.srv import Empty
from std_msgs.msg import Int64
from std_msgs.msg import Bool
import math


class EaterNode(Node):
    def __init__(self):
        super().__init__('eater_node')
        
        self.declare_parameter('max_pizza', 5)
        self.declare_parameter('turtle_eater_name', 'eater_dummy')
        
        self.turtle_eater_name = self.get_parameter('turtle_eater_name').value
        self.max_pizza = self.get_parameter('max_pizza').value

        self.declare_parameter('sampling_frequency', 100)
        self.sampling_frequency = 1 / self.get_parameter('sampling_frequency').value
        
        self.pub_cmdvel = self.create_publisher(Twist, f'/{self.turtle_eater_name}/cmd_vel', 10) 
        self.eat_status = self.create_publisher(Bool, f'/{self.turtle_eater_name}/eat_status', 10)
        
        self.create_subscription(Pose, f'/{self.turtle_eater_name}/pose', self.pose_callback, 10)
        self.create_subscription(Int64, f'/{self.turtle_eater_name}/pizza_count', self.eat_pizza_count_callback, 10)

        self.create_subscription(Point, '/mouse_position', self.mouse_position_callback, 10)

        self.spawn_pizza_client = self.create_client(GivePosition, '/spawn_pizza')
        self.eat_pizza_client = self.create_client(Empty, f'/{self.turtle_eater_name}/eat')

        self.set_max_pizza = self.create_service(SetMaxPizza, f'/{self.turtle_eater_name}/set_max_pizza', self.set_max_pizza_callback)
        self.set_controller_param = self.create_service(SetParam, f'/{self.turtle_eater_name}/set_param', self.set_controller_param_callback)


        self.pizza_cnt = 0
        self.target_queue = []
        
        self.kp_linear = 2.0
        self.kp_angular = 10.0

        self.current_target = None
        self.current_pose = [0.0, 0.0, 0.0]
        self.controller_enable = False
        self.is_eat_all = False
        self.eating = True 
        self.publish_eat_status(True) 
        self.timer = self.create_timer(self.sampling_frequency, self.timer_callback)

    
    
    def publish_eat_status(self, is_eating: bool):
        self.eating = is_eating
        msg = Bool()
        msg.data = is_eating
        self.eat_status.publish(msg)
        
    def spawn_pizza(self, position):
        position_request = GivePosition.Request()
        position_request.x = position[0]
        position_request.y = position[1]
        self.spawn_pizza_client.call_async(position_request)

    def eat_pizza(self):
        self.publish_eat_status(True)
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)

        
        
    def eat_pizza_count_callback(self, msg: Int64):
        self.is_eat_all = msg.data == self.max_pizza
        if msg.data < self.max_pizza:
            self.publish_eat_status(True)  
        else:
            self.publish_eat_status(False)  
                
    def cmd_vel(self, vx, w):
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = w
        self.pub_cmdvel.publish(cmd_vel)

    def mouse_position_callback(self, msg: Point):
        point = [msg.x, msg.y]
        if self.pizza_cnt < self.max_pizza:
            self.target_queue.append(point)
            self.pizza_cnt += 1
            self.spawn_pizza(point)
        elif self.is_eat_all:
            self.target_queue.append(point)
            self.controller_enable = False
            while len(self.target_queue) > 1:
                self.target_queue.pop(0)
        self.get_logger().info(f'Mouse Position: x={msg.x}, y={msg.y}')

    def set_max_pizza_callback(self, request, response):
        new_max = request.max_pizza

        if new_max <= 0:
            self.get_logger().warn(f'Invalid max_pizza: {new_max}')
            response.log = 'failed'
            return response

        if new_max < self.pizza_cnt:
            self.get_logger().warn(
                f'Cannot set max_pizza({new_max}) < current eaten({self.pizza_cnt})'
            )
            response.log = 'failed'
            return response

        self.max_pizza = new_max
        self.is_eat_all = (self.pizza_cnt >= self.max_pizza)
        self.publish_eat_status(not self.is_eat_all)

        response.log = 'success'
        return response
    
    def set_controller_param_callback(self, request, response):
        self.kp_linear = request.kp_linear
        self.kp_angular = request.kp_angular
        self.get_logger().info(f'Set controller params: linear_gain={self.kp_linear}, angular_gain={self.kp_angular}')
        return response


    def pose_callback(self, msg: Pose):
        self.current_pose[0] = msg.x
        self.current_pose[1] = msg.y
        self.current_pose[2] = msg.theta

    def timer_callback(self):

        if len(self.target_queue) > 0 and not self.controller_enable:
            self.current_target = self.target_queue.pop(0)
            self.controller_enable = True
            
            if self.pizza_cnt <= self.max_pizza:
                self.publish_eat_status(True)
            else:
                self.publish_eat_status(False)

        if self.controller_enable:
            dx = self.current_target[0] - self.current_pose[0]
            dy = self.current_target[1] - self.current_pose[1]

            e_dis = math.hypot(dx, dy)
            e_ori = math.atan2(dy, dx) - self.current_pose[2]
            e_ori = math.atan2(math.sin(e_ori), math.cos(e_ori))

            u_dis = self.kp_linear * e_dis
            u_ori = self.kp_angular * e_ori

            if (abs(dx) < 0.1 and abs(dy) < 0.1):
                self.cmd_vel(0.0, 0.0)
                if not self.is_eat_all:
                    self.eat_pizza()
                    self.get_logger().info(f'{self.turtle_eater_name} reached target and is eating')

                else:
                    self.publish_eat_status(False)
                    self.get_logger().info(f'{self.turtle_eater_name} reached target but not eating (max pizza limit reached)')
                self.controller_enable = False
            else:
                self.cmd_vel(u_dis, u_ori)

def main(args=None):
    rclpy.init(args=args)
    node = EaterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()