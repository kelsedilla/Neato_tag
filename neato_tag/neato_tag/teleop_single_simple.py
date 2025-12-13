"""Teleoperation node to control a neato with the user's keyboard inputs."""

# Teleop libraries
import tty
import select
import sys
import termios

# Standard libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TeleopNode(Node):
    """Teleoperation node to control a neato with the user's keyboard inputs.
    WS controls forward and backward, AD rotate the robot in place.
    """

    def __init__(self):
        """Initializes the class."""
        super().__init__("teleopNode")

        # Timer for motors
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

        # Create the publisher for cmd_vel that tells the motors to move.
        self.publisher2 = self.create_publisher(Twist, "neato2/cmd_vel", 10)
        # Define getKey() settings
        self.settings = termios.tcgetattr(sys.stdin)

    def run_loop(self):
        """Key commands for teleoperation robot control"""

        key = self.getKey()
        neato = None
        # Define movement based on key press
        vel = Twist()
        if key == "w":
            vel.linear.x = 0.3
            vel.angular.z = 0.0
        elif key == "s":
            vel.linear.x = -0.3
            vel.angular.z = 0.0
        elif key == "a":
            vel.linear.x = 0.0
            vel.angular.z = 0.5
        elif key == "d":
            vel.linear.x = 0.0
            vel.angular.z = -0.5
        elif key == "c":
            vel.linear.x = 0.0
            vel.angular.z = 0.0
        elif key == "\x03":
            quit()

        self.publisher2.publish(vel)
        key = None

    def getKey(self):
        """Finds current pressed key

        Returns
            String key representing the keyboard key that the user is pressing.
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


def main(args=None):
    """Initialize, run, cleanup."""
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
