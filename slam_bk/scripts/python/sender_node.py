#!/usr/bin/env python
import numpy as np
import rclpy
from rclpy.node import Node
from slam_interfaces.msg import Slamframe
import os
import csv


class MyPublisher(Node):
    def __init__(self):
        super().__init__("my_publisher")
        self.path = "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/saved_data"
        self.file_list = self.list_directories(self.path)
        self.publisher_ = self.create_publisher(Slamframe, "keyframes", 10)
        self.glob_id = 0
        timer_period = 0.3  # seconds
        self.send_int = 0

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def list_directories(self, path):
        # Iterate over all entries in the path
        all_list = []
        for entry in os.listdir(path):
            # Join the path with the entry to get the full path
            full_path = os.path.join(path, entry)
            # Check if it's a directory
            if os.path.isdir(full_path):
                all_list.append(float(entry))

        sorted_doubles = sorted(all_list)

        return sorted_doubles

    def populate_frame(self, path):
        msg = Slamframe()
        # set the mode to send the keyframes
        msg.mode = 1
        msg.id = self.glob_id
        msg.keypoints = []
        tmp_descriptors = []
        msg.descriptors_size = []
        msg.size = []
        msg.angle = []
        msg.response = []
        msg.octave = []
        msg.class_id = []

        point_num = 0

        with open(
            (self.path + "/" + str(path) + "/keypoints_with_descriptors.csv"), mode="r"
        ) as file:
            # Create a CSV reader object
            reader = csv.reader(file)

            # Iterate over each row in the CSV file
            for i, row in enumerate(reader):
                if i == 0:
                    continue

                # keypoints
                msg.keypoints.append(float(row[0]))
                msg.keypoints.append(float(row[1]))

                # size
                msg.size.append(float(row[2]))

                # angle
                msg.angle.append(float(row[3]))

                # response
                msg.response.append(float(row[4]))

                # octave
                msg.octave.append(float(row[5]))

                # class_id
                msg.class_id.append(float(row[6]))

                msg.descriptors_size.append(len(row[7][:-1].split(" ")))

                # descriptors
                float_list = [float(x) for x in row[7][:-1].split()]
                tmp_descriptors = tmp_descriptors + float_list

                point_num = point_num + 1
                msg.point_num = point_num

            msg.descriptors = tmp_descriptors
            msg.timestamp = float(path)
            tmp_s = [1.0, 1.2, 1.44, 1.728, 2.0736, 2.48832, 2.98598, 3.58318]
            msg.scalelevels = tmp_s
            msg.levels = 8

        depth_tmp = []
        with open(
            (self.path + "/" + str(path) + "/depth_information.txt"), "r"
        ) as file:
            for line in file:
                depth_tmp.append(float(line.strip()))
        msg.depth = depth_tmp

        return msg

    def wrapping_up(self):
        msg = Slamframe()
        msg.mode = 2
        return msg

    def populate_int(self):
        msg = Slamframe()
        msg.mode = 0
        # read as string the camera intrinsic path
        intrinsic_path = "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM1.yaml"
        with open(intrinsic_path, "r") as file:
            # Read the contents of the file as a string
            yaml_string = file.read()

        msg.camera_intrisic = yaml_string
        return msg

    def timer_callback(self):
        if self.send_int < 3:
            msg = self.populate_int()
            self.send_int = self.send_int + 1
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg)
        elif self.glob_id < len(self.file_list):
            # Create a new instance of the custom message
            msg = self.populate_frame("{:.6f}".format(self.file_list[self.glob_id]))

            # Publish the message
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.timestamp)

            self.glob_id = self.glob_id + 1
        else:
            msg = self.wrapping_up()
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
