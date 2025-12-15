"""Color tracking code for the Neato Tag version that just makes the chaser neato wait for a bit after a tag before resuming"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from rclpy.qos import qos_profile_sensor_data

from neato_tag.color_detection import color_detection
from neato_tag.color_detection import convert_to_heading_angle


class NeatoTracker(Node):
    """
    The NeatoTracker is a Python object that encompasses a ROS node
    that can process images from the camera and search for an object within.
    The node will issue motor commands to move forward while keeping
    the object in the center of the camera's field of view.
    """

    def __init__(self, image_topic):
        """
        Initialize the NeatoTracker
        """
        super().__init__("neato_tracker")
        self.cv_image = None  # The latest image from the camera
        self.bridge = CvBridge()  # Used to convert ROS messages to OpenCV

        # Subscription for video feed
        self.create_subscription(Image, image_topic, self.process_image, 10)

        # Subscription for bump sensor
        self.create_subscription(
            Bump, "/neato1/bump", self.process_bump, qos_profile=qos_profile_sensor_data
        )

        # Publisher for commands
        self.pub = self.create_publisher(Twist, "neato1/cmd_vel", 10)

        # Variable to keep track of time of bump
        self.bump_time = None

        # How long to wait at bump
        self.bump_duration = 5.0

        # Last known turn direction
        self.turn_direction = 1

        # Bool for if chaser neato should be moving
        self.should_move = True

        # Create initial bounding box of runner neato in image
        self.bounding_box = None
        if not self.cv_image is None:
            self.bounding_box = color_detection(self.cv_image)

        # Create run loop
        self.create_timer(0.1, self.run_loop)

    def process_image(self, msg):
        """
        Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def process_bump(self, msg):
        """
        Trigger stop after bump and start timer
        """
        if msg.left_front or msg.right_front:
            # print("Bump detected! Stopping")

            # Set should move to False and get time of bump
            self.should_move = False
            self.bump_time = time.time()

            # Publish stop command
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.pub.publish(stop_cmd)

    def run_loop(self):
        # If no image detected, don't do anything
        if self.cv_image is None:
            print("No image detected")
            return
        msg_cmd = Twist()

        # Below if statement is to keep track of wait time after bump
        # If neato shouldn't move and a bump has been triggered
        if not self.should_move and self.bump_time is not None:
            # Check amount of time elapsed since time of bump
            elapsed = time.time() - self.bump_time
            # If time elapsed is greater than the bump duration, resume movement
            # and set bump_time back to None
            if elapsed >= self.bump_duration:
                self.should_move = True
                self.bump_time = None
                # print("Resuming movement")

        # If neato should move
        if self.should_move:
            # print("SHOULD MOVE")

            # Get bounding box of the runner neato in the image
            self.bounding_box = color_detection(self.cv_image)

            # If the runner neato is detected
            if self.bounding_box is not None:
                # This commented out section is to show the bounding box in video for debugging
                # cv2.rectangle(
                #     self.cv_image,
                #     (self.bounding_box[0], self.bounding_box[1]),
                #     (self.bounding_box[2], self.bounding_box[3]),
                #     (255, 0, 0),
                #     3,
                # )

                # Calculate the angle of the center of the bounding box to use as heading
                center_angle = convert_to_heading_angle(
                    self.bounding_box, self.cv_image.shape[1]
                )
                # print(f"CENTER ANGLE: {center_angle}")

                # Store the direction of the current turn
                self.turn_direction = np.sign(-center_angle)

                # Set linear and angular velocity to track the runner neato
                msg_cmd.linear.x = 0.5
                msg_cmd.angular.z = -center_angle * 0.03

            # If no bounding box of the runner neato is detected in the image,
            # the chaser neato will keep turning in the last know turn direction
            # it was going in to see if it can find the runner neato
            else:
                # If turn direction is 0, set it to 1
                if self.turn_direction == 0.0:
                    self.turn_direction = 1.0
                msg_cmd.linear.x = 0.1
                msg_cmd.angular.z = self.turn_direction * 1.0
        # If the neato shouldn't move, set linear and angular velocity to 0
        else:
            # print("SHOULD NOT MOVE")
            msg_cmd.linear.x = 0.0
            msg_cmd.angular.z = 0.0
        # This commented out section is to show the video feed with the bounding box for debugging
        # cv2.imshow("video_window", self.cv_image)
        # cv2.waitKey(5)
        # print(msg_cmd)

        # Publish velocity commands
        self.pub.publish(msg_cmd)


def main(args=None):
    """
    Initialize, run, and cleanup
    """
    rclpy.init()
    neato = NeatoTracker("neato1/camera/image_raw")

    try:
        rclpy.spin(neato)
    finally:
        neato.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
