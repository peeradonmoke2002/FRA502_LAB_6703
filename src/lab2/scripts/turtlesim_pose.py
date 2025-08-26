#!/usr/bin/python3
# Subscribe to turtle poses from both /turtle1/pose and /turtle2/pose (turtlesim/Pose) to track the positions of both turtles in real-time.
# Map default grid interface with 10x10m dimensions that properly interfaces with the turtlesim+ GUI coordinate system for visualization and interaction.
# Publish odometry data by creating publishers for /odom1 and /odom2 (nav_msgs/Odometry) to provide proper odometry information for both turtle1 and turtle2.
# Broadcast transforms between the odom frame and individual turtle frames (turtle1 and turtle2) using the TF2 system to maintain proper coordinate frame relationships.
# Implement odometry publishing function that handles publishing odometry data to /odom1, /odom2, and corresponding /tf transforms for both turtles, following the example structure: def example_pub(self, msg, turtle_name, child_frame_id).
# RViz2 compatibility by ensuring the node works seamlessly with the provided config fun2.rviz configuration file for proper visualization.
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import tf_transformations as tf
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry


class TurtlesimPoseNode(Node):
    def __init__(self):
        super().__init__('turtlesim_pose_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.turtlename1 = self.declare_parameter(
          'turtlename1', 'turtle1').get_parameter_value().string_value
        self.turtlename2 = self.declare_parameter(
          'turtlename2', 'turtle2').get_parameter_value().string_value
        self.odom_pub_1 = self.create_publisher(Odometry, '/odom1', 10)
        self.odom_pub_2 = self.create_publisher(Odometry, '/odom2', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb_1, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.pose_cb_2, 10)
        self.turtle1 = Pose()
        self.turtle2 = Pose()


        # Control loop timer
        self.create_timer(0.05, self.on_timer)
        
    def pose_cb_1(self, msg):
        self.turtle1 = msg

    def pose_cb_2(self, msg):
        self.turtle2 = msg

        
    def tf_odom_pub(self, msg, odom_pub, child_frame_id):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = child_frame_id
        odom_msg.pose.pose.position.x = msg.x
        odom_msg.pose.pose.position.y = msg.y
        odom_msg.pose.pose.position.z = 0.0
        q = tf.quaternion_from_euler(0, 0, msg.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_pub.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = child_frame_id
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_msg.pose.pose.orientation.x
        t.transform.rotation.y = odom_msg.pose.pose.orientation.y
        t.transform.rotation.z = odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = odom_msg.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)
    
        
    def on_timer(self):
        self.tf_odom_pub(self.turtle1, self.odom_pub_1, self.turtlename1)
        self.tf_odom_pub(self.turtle2, self.odom_pub_2, self.turtlename2)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
