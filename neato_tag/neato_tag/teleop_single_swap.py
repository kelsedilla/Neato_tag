"""Teleoperation node to control a neato with the user's keyboard inputs. This is for the Neato Tag version that swaps"""

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
    """
    Teleoperation node to control a neato with the user's keyboard inputs.
    WS controls forward and backward, AD rotate the robot in place.
    """

    def __init__(self):
        """Initializes the class."""
        super().__init__("teleop_node")

        # Timer for motors
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

        # Dict to store chaser and runner neato. color_tracking is the chaser and teleop is runner
        # Chaser starts as neato1 and runner starts as neato2
        self.roles = {"color_tracking": "neato1", "teleop": "neato2"}

        # Subscribe to swap roles topic (the chaser neato will publish to this when it sense it tags)
        self.swap_sub = self.create_subscription(
            Empty,
            "/swap_roles",
            self.swap,
            qos_profile=qos_profile_sensor_data,
        )

        # Call method to setup publishers
        self.setup()

        # Key input
        self.settings = termios.tcgetattr(sys.stdin)
        self.current_key = ""  # Stores the latest key press
        self.running = True

        # Start a separate thread for reading keyboard input
        # Created separate thread because previous method of key reading would block
        # things until a key input was read
        self.key_thread = threading.Thread(target=self.key_reader)
        self.key_thread.daemon = True  # Thread exits when main program exits
        self.key_thread.start()

    def setup(self):
        """
        Method to setup the publisher for the runner neato.
        Destroys any previous publishers for clean setup
        """
        # If there is already a publisher for commands, destroy it
        if hasattr(self, "pub"):
            self.destroy_publisher(self.pub)

        # Get current runner neato name
        neato = self.roles["teleop"]

        # Create publisher for commands
        self.pub = self.create_publisher(Twist, f"{neato}/cmd_vel", 10)

    def swap(self, msg):
        """
        Method to swap roles of chaser and runner neato
        """
        # print("SWAPPING ROLES")
        # print(self.roles)
        self.roles["color_tracking"], self.roles["teleop"] = (
            self.roles["teleop"],
            self.roles["color_tracking"],
        )

        # Re-setup the publisher for the runner
        self.setup()

    def run_loop(self):
        """Key commands for teleoperation robot control"""
        key = self.current_key  # Get latest key press

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
            # Don't call quit(). The main loop will exit on ctrl+c
            return

        self.pub.publish(vel)
        key = None

    def key_reader(self):
        """
        Reads keyboard input in a non-blocking way

        This runs in a separate thread so keyboard input does not block
        the ROS timer or spin loop
        """
        # Put terminal into raw mode to get single key presses
        tty.setraw(sys.stdin.fileno())

        try:
            while self.running:
                # Select() checks if stdin has input available
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

                # If there's a key press, read it
                if rlist:
                    key = sys.stdin.read(1)

                    # If ctrl+c pressed
                    if key == "\x03":
                        self.stop()
                        break

                    # Store the latest key press for run_loop() to use
                    self.current_key = key
        finally:
            # Restore terminal settings when thread exits
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def stop(self):
        """
        This stops the key reading thread and shuts down rclpy. This ensures
        everything shuts down cleanly and terminal does not get stuck
        """
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
