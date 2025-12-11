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

from color_detection import color_detection
from color_detection import convert_to_lidar_angles

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

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.center_x = 0
        self.center_y = 0
        self.turn_direction = 1
        self.should_move = False

        # create initial bounding box
        self.bounding_box = None
        self.confidence = 0
        if not self.cv_image is None:
            self.bounding_box = color_detection(self.cv_image)
            self.confidence = 1

        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_image(self, msg):
        """Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing"""
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def loop_wrapper(self):
        """This function takes care of calling the run_loop function repeatedly.
        We are using a separate thread to run the loop_wrapper to work around
        issues with single threaded executors in ROS2"""
        # cv2.namedWindow("video_window")
        print(f"confidence: {self.confidence}")
        if self.cv_image is None:
            print("None")
        while True:
            self.run_loop()
            time.sleep(0.1)

    def track_neato(self):
        print(self.bounding_box)
        if not self.bounding_box is None:
            neato = self.tracker.update_tracks(
                [[self.bounding_box, self.confidence, "color_detection"]],
                frame=self.cv_image,
            )[0]
            self.confidence = max(0, self.confidence - 0.001)
            ltrb = neato.to_ltrb()
            self.bounding_box = [int(ltrb[0]), int(ltrb[1]), int(ltrb[2]), int(ltrb[3])]

    def run_loop(self):
        # NOTE: only do cv2.imshow and cv2.waitKey in this function
        # if self.confidence > 0.99:
        #     self.track_neato()
        if not self.cv_image is None:
            self.bounding_box = color_detection(self.cv_image)
            self.should_move = True
        if not self.cv_image is None:
            print(self.bounding_box)
            msg_cmd = Twist()
            if not self.bounding_box is None:
                xmin, ymin, xmax, ymax = self.bounding_box
                left_angle, right_angle = convert_to_lidar_angles(
                    self.bounding_box, self.cv_image.shape[1]
                )
                center_angle = (left_angle + right_angle) / 2
                self.center_x = (xmin + xmax) / 2.0
                self.center_y = (ymin + ymax) / 2.0
                self.turn_direction = np.sign(-center_angle)
                if self.should_move is True:
                    msg_cmd.linear.x = 0.5
                    msg_cmd.angular.z = -center_angle * 0.03
                    print(center_angle)
            elif self.should_move is True:
                msg_cmd.linear.x = 0.1
                msg_cmd.angular.z = self.turn_direction * 1.0
                print(msg_cmd.angular.z)
            self.pub.publish(msg_cmd)

            if not self.bounding_box is None:
                cv2.rectangle(
                    self.cv_image,
                    (self.bounding_box[0], self.bounding_box[1]),
                    (self.bounding_box[2], self.bounding_box[3]),
                    (255, 0, 0),
                    3,
                )
            # cv2.imshow("video_window", self.cv_image)
            # cv2.waitKey(5)


def main(args=None):
    rclpy.init()
    n = NeatoTracker("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


###################################################################################################


# def create_video_writer(video_cap, output_filename):
#     # Function to create a video writer object for saving the output video
#     frame_width = int(video_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
#     frame_height = int(video_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
#     fps = int(video_cap.get(cv2.CAP_PROP_FPS))
#     fourcc = cv2.VideoWriter_fourcc(*'MP4V')
#     writer = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))
#     return writer

# CONFIDENCE_THRESHOLD = 0.8  # Confidence threshold for detecting objects
# GREEN = (0, 255, 0)  # Color for drawing bounding boxes
# WHITE = (255, 255, 255)  # Color for drawing text

# video_cap = cv2.VideoCapture("/content/video.mp4")  # Initialize the video capture object to read the video
# writer = create_video_writer(video_cap, "output.mp4")  # Initialize the video writer object to save the processed video

# model = YOLO("yolov8n.pt")  # Load the pre-trained YOLOv8n model
# tracker = DeepSort(max_age=50)  # Initialize the DeepSORT tracker

# while True:
#     start = datetime.datetime.now()  # Record the start time

#     ret, frame = video_cap.read()  # Read a frame from the video
#     if not ret:
#         break  # Exit the loop if no frame is read

#     detections = model(frame)[0]  # Run the YOLO model on the frame to detect objects
#     results = []

#     for data in detections.boxes.data.tolist():
#         confidence = data[4]  # Extract the confidence level of the detection
#         if float(confidence) < CONFIDENCE_THRESHOLD:
#             continue  # Ignore detections with low confidence

#         # Get the bounding box coordinates and class ID
#         xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
#         class_id = int(data[5])
#         results.append([[xmin, ymin, xmax - xmin, ymax - ymin], confidence, class_id])


#     tracks = tracker.update_tracks(results, frame=frame)
#     for track in tracks:
#         if not track.is_confirmed():
#             continue  # Ignore unconfirmed tracks

#         track_id = track.track_id  # Get the track ID
#         ltrb = track.to_ltrb()  # Get the bounding box coordinates
#         xmin, ymin, xmax, ymax = int(ltrb[0]), int(ltrb[1]), int(ltrb[2]), int(ltrb[3])
#         # Draw the bounding box and the track ID on the frame
#         cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
#         cv2.rectangle(frame, (xmin, ymin - 20), (xmin + 20, ymin), GREEN, -1)
#         cv2.putText(frame, str(track_id), (xmin + 5, ymin - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 2)

#     end = datetime.datetime.now()  # Record the end time
#     print(f"Time to process 1 frame: {(end - start).total_seconds() * 1000:.0f} milliseconds")
#     fps = f"FPS: {1 / (end - start).total_seconds():.2f}"  # Calculate and display the FPS
#     cv2.putText(frame, fps, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 8)

#     # Display the frame and write it to the output video
#     cv2_imshow(frame)
#     writer.write(frame)
#     if cv2.waitKey(1) == ord("q"):
#         break  # Exit the loop if 'q' key is pressed

# video_cap.release()  # Release the video capture object
# writer.release()  # Release the video writer object
# cv2.destroyAllWindows()  # Close all OpenCV windows
