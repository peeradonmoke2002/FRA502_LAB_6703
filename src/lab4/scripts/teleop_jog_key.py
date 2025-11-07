#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import select
import termios
import tty

# Instructions displayed to user
msg = """
Control Your Robot Arm!
---------------------------
Moving around:
   w
 a s d

w/s : increase/decrease x velocity
a/d : increase/decrease y velocity
q/e : increase/decrease z velocity

f : toggle frame (world/tool)
r : reset to initial pose
space : stop all motion
x : exit

Current velocities will be displayed
"""

# Key mappings for velocity control
moveBindings = {
    'w': (1, 0, 0),   # +X
    's': (-1, 0, 0),  # -X
    'a': (0, 1, 0),   # +Y
    'd': (0, -1, 0),  # -Y
    'q': (0, 0, 1),   # +Z
    'e': (0, 0, -1),  # -Z
}

def getKey(settings):
    """Get a single keypress from terminal"""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopJogKey(Node):
    def __init__(self):
        super().__init__('teleop_jog_key')

        # Publisher for cmd_vel
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for frame switching
        self.frame_pub = self.create_publisher(String, '/teleop_frame', 10)

        # Publisher for reset command
        self.reset_pub = self.create_publisher(String, '/reset_pose', 10)

        # Velocity settings
        self.speed = 0.1  # m/s - linear speed increment
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # Frame mode
        self.frame_mode = "tool"  # "tool" or "world"

        self.get_logger().info("Teleop Jog Key node started")

    def publish_velocity(self, vx, vy, vz):
        """Publish Twist message with given velocities"""
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)

    def stop(self):
        """Stop all motion"""
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.publish_velocity(0.0, 0.0, 0.0)

    def toggle_frame(self):
        """Toggle between world and tool frame"""
        if self.frame_mode == "tool":
            self.frame_mode = "world"
        else:
            self.frame_mode = "tool"

        # Publish frame change
        frame_msg = String()
        frame_msg.data = self.frame_mode
        self.frame_pub.publish(frame_msg)
        return self.frame_mode

    def reset_pose(self):
        """Send reset command to controller"""
        reset_msg = String()
        reset_msg.data = "reset"
        self.reset_pub.publish(reset_msg)
        self.stop()  # Also stop all motion


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJogKey()

    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)

    print(msg)
    print(f"Speed: {node.speed} m/s")
    print(f"Frame: {node.frame_mode}")

    try:
        while True:
            key = getKey(settings)

            if key in moveBindings.keys():
                # Update velocities based on key
                dx, dy, dz = moveBindings[key]
                node.vx = dx * node.speed
                node.vy = dy * node.speed
                node.vz = dz * node.speed

                # Publish velocity
                node.publish_velocity(node.vx, node.vy, node.vz)

                print(f"Velocity: vx={node.vx:.3f}, vy={node.vy:.3f}, vz={node.vz:.3f} | Frame: {node.frame_mode}")

            elif key == ' ':
                # Stop
                node.stop()
                print("Stopped")

            elif key == 'f':
                # Toggle frame
                new_frame = node.toggle_frame()
                print(f"Frame switched to: {new_frame}")

            elif key == 'r':
                # Reset to initial pose
                node.reset_pose()
                print("Resetting to initial pose...")

            elif key == 'x':
                # Exit
                print("Exiting...")
                break

            else:
                # Invalid key
                if key == '\x03':  # Ctrl+C
                    break

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Stop the robot and restore terminal
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
