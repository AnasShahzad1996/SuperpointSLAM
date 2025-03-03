import os
import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from slam_interfaces.msg import Slamframe

class FeatureExtractor(Node):
    def __init__(
        self, node_id, mode, initrinsic_file, extractor_type, association_file=None
    ):
        super().__init__("my_publisher")
        self.mode = mode
        self.initrinsic_file = initrinsic_file
        self.extractor_type = extractor_type
        self.association_file = association_file
        #self.depth_scale = 5000
        self.depth_scale = 255
        self.timer_period = 0.1
        self.publisher_ = self.create_publisher(Slamframe, "keyframes", 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.frame_id = 0
        self.send_int = 0
        self.wrapping_up_c = 0

        # if offline
        self.rgb_files = []
        self.depth_files = []
        if self.mode == "offline":
            self.populate_rgbd_files()

    def populate_rgbd_files(self):
        with open(self.association_file, "r") as file:
            for line in file:
                contents = line.split(" ")
                print(contents)
                self.rgb_files.append(contents[1])
                self.depth_files.append(contents[3][:-1])

    def read_images(self):
        data_path = ("/").join(self.association_file.split("/")[:-1]) + "/"
        rgb_image = cv2.transpose(
            cv2.imread(data_path + self.rgb_files[self.frame_id], cv2.IMREAD_GRAYSCALE)
        )
        depth_image = cv2.imread(
            data_path + self.depth_files[self.frame_id], cv2.IMREAD_UNCHANGED
        )
        timestamp = self.rgb_files[self.frame_id].split("/")[1][:-4]
        return rgb_image, depth_image, timestamp

    def orb_extractor(self, rgb_file=None, depth_file=None, timestamp=None):
        n_levels = 8  # Number of scale levels
        scale_factor = 1.2  # Scale factor between levels in the pyramid
        orb = cv2.ORB_create(nfeatures=1000, scaleFactor=1.2, nlevels=8)

        msg = Slamframe()
        if timestamp[-1] == ".":
            msg.timestamp = float(timestamp[:-1])
        else:
            msg.timestamp = float(timestamp)
        tmp_s = [1.0, 1.2, 1.44, 1.728, 2.0736, 2.48832, 2.98598, 3.58318]
        msg.scalelevels = tmp_s
        msg.levels = 8
        msg.mode = 1
        msg.id = self.frame_id
        msg.keypoints = []
        tmp_descriptors = []
        msg.descriptors_size = []
        msg.size = []
        msg.angle = []
        msg.response = []
        msg.octave = []
        msg.class_id = []
        point_num = 0

        tmp_des = []
        tmp_depth = []
        keypoints, descriptors = orb.detectAndCompute(rgb_file, None)
        for i, currk in enumerate(keypoints):
            # Add the keypoints
            x, y = currk.pt
            msg.keypoints.append(y)
            msg.keypoints.append(x)

            # kp.size, kp.angle, kp.response, kp.octave, kp.class_id
            msg.size.append(currk.size)
            msg.angle.append(currk.angle)
            msg.response.append(currk.response)
            msg.octave.append(currk.octave)
            msg.class_id.append(currk.class_id)

            # Add descriptors
            msg.descriptors_size.append(len(descriptors[i]))
            tmp_des = tmp_des + list(descriptors[i].astype(float))
            depth_info = depth_file[int(x), int(y)] / self.depth_scale
            tmp_depth.append(float(depth_info))
            point_num = point_num + 1
        msg.point_num = point_num
        msg.descriptors = tmp_des
        msg.depth = tmp_depth

        return msg

    def gcn_extractor(self, rgb_file=None, depth_file=None):
        msg = Slamframe()

    def populate_int(self):
        msg = Slamframe()
        msg.mode = 0
        # read as string the camera intrinsic path
        with open(self.initrinsic_file, "r") as file:
            # Read the contents of the file as a string
            yaml_string = file.read()

        msg.camera_intrisic = yaml_string
        return msg

    def wrapping_up(self):
        msg = Slamframe()
        msg.mode = 2
        self.wrapping_up_c = self.wrapping_up_c + 1
        return msg

    def timer_callback(self):
        msg = None
        if self.send_int < 3:
            msg = self.populate_int()
            self.send_int = self.send_int + 1
            self.publisher_.publish(msg)
            #self.get_logger().info('Publishing: "%s"' % msg)
        elif self.frame_id < len(self.rgb_files):
            rgb_image, depth_image, timestamp = self.read_images()
            if self.extractor_type == "orb":
                msg = self.orb_extractor(
                    rgb_file=rgb_image, depth_file=depth_image, timestamp=timestamp
                )
            elif self.extractor_type == "gcn":
                msg = self.gcn_extractor(rgb_file=rgb_image, depth_file=depth_image)

            self.publisher_.publish(msg)
            self.frame_id = self.frame_id + 1
            print("Publishing ", msg.id)
        else:
            msg = self.wrapping_up()
            self.publisher_.publish(msg)
            if self.wrapping_up_c > 3:
                self.get_logger().info("Published all frames..")
                raise SystemExit
