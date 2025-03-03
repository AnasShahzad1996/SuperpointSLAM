import os
import cv2
import sys
import rclpy
import struct
import argparse
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from PIL import Image
from associate import *
from evaluate_rpe import *
from generate_pointc import *
from rclpy.node import Node
from slam_interfaces.msg import Posemsg


class PoseReceiver(Node):
    def __init__(self, pose_file, dataset_path, association_file):
        super().__init__("my_publisher")
        print("This is the pose receiver")
        self.mode = 0
        self.count = 0
        self.subscription = self.create_subscription(
            Posemsg,  # Replace YourMessageType with your actual message type
            "pose",  # Replace your_topic_name with the name of the topic you want to subscribe to
            self.callback,
            10,  # Adjust the queue size according to your needs
        )
        print("Pose receiver")

    def callback(self, msg):
        if self.count > 5:
            raise SystemExit

        self.count = self.count + 1
        self.get_logger().info(
            "Received: %s" % msg.id
        )  # Replace msg.data with the appropriate field of your message

        counti = 0
        with open("CameraTrajectoryR.txt", "w") as file:
            for i, timestamp in enumerate(msg.timestamp):
                start_ind = counti
                end_ind = counti + 7
                curr_msg = (
                    str(timestamp)
                    + " "
                    + " ".join(map(str, msg.posequat[start_ind:end_ind]))
                    + "\n"
                )
                counti = counti + 7
                file.write(curr_msg)


class ReconstructPointCloud:
    def __init__(self, camera_traj, association_file, global_path):
        self.global_path = global_path
        self.association_file = association_file
        self.camera_traj = camera_traj
        self.rgb_files, self.depth_files, self.timestamps_rgb, self.timestamps_depth = [],[],[],[]
        self.populate_rgbd_files()

        self.points = []
        self.merge_p()

    def sgd_reconstruct(self):
        """Use SGD reconstruction to hallucinate the pointcloud"""


    def merge_p(self):
        pose_list = read_file_list(self.camera_traj)


        rgb_dicti = {float(self.timestamps_rgb[i]): [self.rgb_files[i]] for i in range(len(self.timestamps_rgb))}
        depth_dicti = {float(self.timestamps_depth[i]): [self.depth_files[i]] for i in range(len(self.timestamps_depth))}

        matches_rgb_depth = dict(associate(rgb_dicti, depth_dicti, 0.0, 0.02))
        matches_rgb_traj = associate(matches_rgb_depth, pose_list, 0.00, 0.01)
        matches_rgb_traj.sort()

        traj = read_trajectory(self.camera_traj)

        all_points = []
        list  = range(0,len(matches_rgb_traj),1)
        for frame,i in enumerate(list):
            rgb_stamp,traj_stamp = matches_rgb_traj[i]

            rgb_file = rgb_dicti[rgb_stamp][0]
            depth_file = depth_dicti[matches_rgb_depth[rgb_stamp]][0]
            pose = traj[traj_stamp]

            points = generate_pointcloud(rgb_file,depth_file,pose,8, False)

            all_points += points
            print ("Frame %d/%d, number of points so far: %d"%(frame+1,len(list),len(all_points)))

        write_ply("reconstP.ply",all_points)

    def populate_rgbd_files(self):
        with open(self.association_file, "r") as file:
            for line in file:
                contents = line.split(" ")
                self.timestamps_rgb.append(contents[0])
                self.rgb_files.append(self.global_path+"/"+contents[1])
                self.timestamps_depth.append(contents[2])
                self.depth_files.append(self.global_path+"/"+contents[3][:-1])
