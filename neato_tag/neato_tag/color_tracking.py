import datetime  # Library for handling date and time operations

# from ultralytics import YOLO  # Library for loading and using the YOLO model
import time
from threading import Thread
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from rclpy.qos import qos_profile_sensor_data

from neato_tag.color_detection import color_detection
from neato_tag.color_detection import convert_to_heading_angle

RED_LOWER_BOUND = 0
GREEN_LOWER_BOUND = 0
BLUE_LOWER_BOUND = 0
RED_UPPER_BOUND = 255
GREEN_UPPER_BOUND = 255
BLUE_UPPER_BOUND = 255


class NeatoTracker(Node):
    """
    The NeatoTracker is a Python object that encompasses a ROS node
    that can process images from the camera and search for an object within.
    The node will issue motor commands to move forward while keeping
    the object in the center of the camera's field of view.
    """

    def __init__(self, image_topic):
        """Initialize the NeatoTracker"""
        super().__init__("neato_tracker")
        self.cv_image = None  # the latest image from the camera
        self.bridge = CvBridge()  # used to convert ROS messages to OpenCV

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.create_subscription(
            Bump, "/neato1/bump", self.process_bump, qos_profile=qos_profile_sensor_data
        )

        self.pub = self.create_publisher(Twist, "neato1/cmd_vel", 10)
        self.bump_time = None
        self.bump_duration = 5.0
        self.turn_direction = 1
        self.should_move = True

        # create initial bounding box
        self.bounding_box = None
        if not self.cv_image is None:
            self.bounding_box = color_detection(self.cv_image)

        self.create_timer(0.1, self.run_loop)
        # thread = Thread(target=self.loop_wrapper)
        # thread.start()

    def process_image(self, msg):
        """Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing"""
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def process_bump(self, msg):
        """Trigger stop and start debounce timer."""
        if msg.left_front or msg.right_front:
            print("Bump detected! Stopping")
            self.should_move = False
            self.bump_time = time.time()

            # immediately publish stop command
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.pub.publish(stop_cmd)

    # def loop_wrapper(self):
    #     """This function takes care of calling the run_loop function repeatedly.
    #     We are using a separate thread to run the loop_wrapper to work around
    #     issues with single threaded executors in ROS2"""
    #     # cv2.namedWindow("video_window")
    #     while True:
    #         if self.cv_image is None:
    #             print("None")
    #         else:
    #             self.run_loop()
    #             time.sleep(0.1)

    def run_loop(self):
        if self.cv_image is None:
            print("No image detected")
            return
        msg_cmd = Twist()
        if not self.should_move and self.bump_time is not None:
            elapsed = time.time() - self.bump_time
            if elapsed >= self.bump_duration:
                self.should_move = True
                self.bump_time = None
                print("Resuming movement")
        if self.should_move:
            print("SHOULD MOVE")
            self.bounding_box = color_detection(self.cv_image)
            if self.bounding_box is not None:
                # This commented out section is to show the bounding box in video for debugging
                # cv2.rectangle(
                #     self.cv_image,
                #     (self.bounding_box[0], self.bounding_box[1]),
                #     (self.bounding_box[2], self.bounding_box[3]),
                #     (255, 0, 0),
                #     3,
                # )
                center_angle = convert_to_heading_angle(
                    self.bounding_box, self.cv_image.shape[1]
                )
                print(f"CENTER ANGLE: {center_angle}")
                self.turn_direction = np.sign(-center_angle)
                msg_cmd.linear.x = 0.5
                msg_cmd.angular.z = -center_angle * 0.03
            else:
                msg_cmd.linear.x = 0.1
                msg_cmd.angular.z = self.turn_direction * 1.0
        else:
            print("SHOULD NOT MOVE")
            msg_cmd.linear.x = 0.0
            msg_cmd.angular.z = 0.0
        # This commented out section is to show the video feed with the bounding box for debugging
        # cv2.imshow("video_window", self.cv_image)
        # cv2.waitKey(5)
        print(msg_cmd)
        self.pub.publish(msg_cmd)


def main(args=None):
    rclpy.init()
    n = NeatoTracker("neato1/camera/image_raw")
    rclpy.spin(n)
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
