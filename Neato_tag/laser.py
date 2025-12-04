"""
This is a person follower node, which directs the Neato to follow a nearby person.
"""

import math
import statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt


class PersonFollowerNode(Node):
    """
    A Node subclass that makes the Neato follow a person.
    """

    LASER_SCAN_RANGE_MIN = 0.01  # meters
    LASER_SCAN_RANGE_MAX = 5.0  # meters
    LASER_SCAN_ARRAY_LENGTH = 361
    Kp_angular = 0.02
    Kp_linear = 0.5
    _distances = [0.0] * LASER_SCAN_ARRAY_LENGTH
    _filter_distance = 1.5
    _person_follow_point_cutoff = 1

    def __init__(self):
        super().__init__("person_follower")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(
            LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data
        )

    def run_loop(self):
        """
        Run the person following behavior.

        Publishes:
            Twist representing the desired linear and angular velocities to the /cmd_vel
            topic.
        """
        # print(self._distances)
        x_val, y_val = convert_to_cartesian(self._distances)
        plot_cartesian(x_val, y_val)
        # [person_angle, person_distance] = self.find_person()
        # msg.angular.z = person_angle * self.Kp_angular
        # msg.linear.x = min(person_distance * self.Kp_linear, 0.3)
        # print([msg.angular.z, msg.linear.x])

        # self.vel_pub.publish(msg)

    def process_scan(self, msg):
        """
        This function is called whenever a new lidar scan is published on the /scan
        topic. It creates a distances and angles array, which contain the distance
        from the Neato and angle (starting from the front of the Neato and going
        counterclockwise) of each point in the scan. If the resulting distance is
        out of the Neato lidar's scan range, the value becomes None.

        Args:
            msg: A LaserScan message, which is passed in by the subscription.

        Returns:
            distances: An array of floats containing the distance in meters of each
                       point in the scan.
            angles: An array of floats containing the angle (in degrees, going
                    counterclockwise from the front of the Neato) of each point in
                    the scan.
        """
        for i, distance in enumerate(msg.ranges):
            if self.LASER_SCAN_RANGE_MIN < distance < self.LASER_SCAN_RANGE_MAX:
                self._distances[i] = distance
            else:
                self._distances[i] = 0

    def find_person(self):
        """
        This function locates where a person is relative to the Neato, assuming they
        are the only large object within a radius of 2 meters.

        Args:
            distance_array: an array of floats representing the distance from the Neato to
            each point near it

        Returns:
            person_angle: a float representing the person's angle in degrees
            person_distance: a float representing the person's distance in degrees
        """
        distance_array = [
            0 if d > self._filter_distance else d for d in self._distances
        ]
        # print(distance_array)
        person_point_angles = []
        person_point_distance = []
        for angle, distance in enumerate(distance_array):
            if distance > 0:
                person_point_angles.append(angle)
                person_point_distance.append(distance)

        if len(person_point_angles) < self._person_follow_point_cutoff:
            return 0.0, 0.0
        person_point_angles = convert_to_signed_angles(person_point_angles)

        person_angle = float(statistics.mean(person_point_angles))
        person_distance = float(statistics.mean(person_point_distance))
        return person_angle, person_distance


def convert_to_signed_angles(angle_list):
    """
    This function converts a list of angles from [0, 360) to [-180, 180).

    Args:
        angle_list: a list of integers representing angles from 0 to 360 degrees

    Returns:
        a list of integers representing the input angles centered about 0
    """
    return [(angle + 180) % 360 - 180 for angle in angle_list]


def convert_to_cartesian(distances):
    """
    This function converts the points detected in the lidar scan from polar 
    coordinates to cartesian coordinates. Used in the debugging process.

    Args:
        distances: a list of floats containing the distances, in meters, to
        the nearest point corresponding with a specific angle

    Returns:
        x_val: a list of floats containing the x coordinate of each point
        y_val: a list of floats containing the y coordinate of each point
    """
    x_val = []
    y_val = []
    for i, d in enumerate(distances):
        angle = math.radians(i)
        x = d * math.cos(angle)
        y = d * math.sin(angle)
        x_val.append(x)
        y_val.append(y)

    return x_val, y_val


def plot_cartesian(x_val, y_val, person_x=None, person_y=None):
    """
    This function creates a plot that marks each point detected by the lidar scan
    in blue and the estimated location of a person in red. Used in the debugging
    process.

    Args:
        x_val: a list of floats containing the x coordinate of each point
        y_val: a list of floats containing the y coordinate of each point
        person_x: a float representing the x coordinate of a person's estimated
                  location
        person_y: a float representing the y coordinate of a person's estimated
                  location

    Returns:
        does not return anything
    """
    plt.figure(figsize=(6, 6))
    plt.scatter(x_val, y_val, s=5, c="blue")
    if person_x is not None and person_y is not None:
        plt.scatter(person_x, person_y, color="red", s=50, label="Estimated Person")
    plt.axis("equal")
    plt.grid(True)
    plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    print("shutdown")


if __name__ == "__main__":
    main()
