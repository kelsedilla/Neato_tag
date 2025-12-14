"""Teleoperation node to control a neato with the user's keyboard inputs."""

# Teleop libraries
import tty
import select
import sys
import termios
import threading

# Standard libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from rclpy.qos import qos_profile_sensor_data


class TeleopNode(Node):
    """Teleoperation node to control a neato with the user's keyboard inputs.
    WS controls forward and backward, AD rotate the robot in place.
    """

    def __init__(self):
        """Initializes the class."""
        super().__init__("teleop_node")

        # Timer for motors
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

        self.roles = {"color_tracking": "neato1", "teleop": "neato2"}

        self.swap_sub = self.create_subscription(
            Empty,
            "/swap_roles",
            self.swap,
            qos_profile=qos_profile_sensor_data,
        )
        self.setup()

        # Key input
        self.settings = termios.tcgetattr(sys.stdin)
        self.current_key = ""  # stores the latest key press
        self.running = True

        # Start a separate thread for reading keyboard input
        self.key_thread = threading.Thread(target=self.key_reader)
        self.key_thread.daemon = True  # thread exits when main program exits
        self.key_thread.start()

    def setup(self):
        if hasattr(self, "pub"):
            self.destroy_publisher(self.pub)
        neato = self.roles["teleop"]
        self.pub = self.create_publisher(Twist, f"{neato}/cmd_vel", 10)

    def swap(self, msg):
        print("SWAPPING ROLES")
        print(self.roles)
        self.roles["color_tracking"], self.roles["teleop"] = (
            self.roles["teleop"],
            self.roles["color_tracking"],
        )
        self.setup()

    def run_loop(self):
        """Key commands for teleoperation robot control"""
        key = self.current_key  # get latest key press
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
            # Don't call quit(); the main loop will exit on Ctrl+C
            return

        self.pub.publish(vel)
        key = None

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        key = ""
        if rlist:
            key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def key_reader(self):
        tty.setraw(sys.stdin.fileno())
        try:
            while self.running:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                    if key == "\x03":  # Ctrl+C
                        self.stop()
                        break
                    self.current_key = key
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def stop(self):
        """Safely stop the node and keyboard thread"""
        self.running = False
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()


if __name__ == "__main__":
    main()
