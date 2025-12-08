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
from deep_sort_realtime.deepsort_tracker import DeepSort  # Library for the DeepSORT tracker

from Neato_tag.color_detection import color_detection

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
        """ Initialize the NeatoTracker """
        super().__init__("neato_tracker")
        self.cv_image = None        # the latest image from the camera
        self.bridge = CvBridge()    # used to convert ROS messages to OpenCV

        self.create_subscription(Image, image_topic, self.process_image, 10)

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.center_x = 0
        self.center_y = 0
        self.turn_direction = 1
        self.should_move = False

        # create initial bounding box
        self.ltrb = None

        self.tracker = DeepSort(max_age=50)  # Initialize the DeepSORT tracker

        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        if self.cv_image is None:
            print("None")
        while True:
            self.run_loop()
            time.sleep(0.1)

    def track_neato(self, bounding_box):
        if not bounding_box is None:
            x = bounding_box[0]
            y = bounding_box[1]
            w = bounding_box[2] - bounding_box[0]
            h = bounding_box[3] - bounding_box[1]

            dets = [[[x, y, w, h], 1.0]]
            tracks = self.tracker.update_tracks(dets, frame=self.cv_image)
        else:
            dets = np.empty((0, 5))
            tracks = self.tracker.update_tracks(dets, frame=self.cv_image)
        for i, track in enumerate(tracks):
            if i == 0:
                self.ltrb = track.to_ltrb()
            if not track.is_confirmed():
                continue
            l, t, r, b = track.to_ltrb()
            cv2.rectangle(self.cv_image, (int(l), int(t)), (int(r), int(b)), (255, 0, 0), 2)

    def run_loop(self):
        # NOTE: only do cv2.imshow and cv2.waitKey in this function
        if not self.cv_image is None:
            bounding_box = color_detection(self.cv_image)

            self.track_neato(bounding_box)
            self.should_move = True
            msg_cmd = Twist()
            print(self.ltrb)
            if not self.ltrb is None:
                l, t, r, b = self.ltrb
                self.center_x = (l + r) / 2.0
                self.center_y = (t + b) / 2.0
                # normalize self.center_x to range roughly [-1, 1]
                norm_x_pose = (self.center_x - (self.cv_image.shape[1] / 2.0)) / (self.cv_image.shape[1] / 2.0)
                self.turn_direction = np.sign(-norm_x_pose)
                # create message pose (stopped, else move towards target)
                if self.should_move is True:
                    msg_cmd.linear.x = 0.1
                    msg_cmd.angular.z = -norm_x_pose
            elif self.should_move is True:
                msg_cmd.linear.x = 0.1
                msg_cmd.angular.z = self.turn_direction * 1.0
            self.pub.publish(msg_cmd)

            cv2.imshow('video_window', self.cv_image)
            cv2.waitKey(5)

def main(args=None):
    rclpy.init()
    n = NeatoTracker("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

