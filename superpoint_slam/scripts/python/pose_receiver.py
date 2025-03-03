import os
import cv2
import sys
import json
import rclpy
import struct
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from PIL import Image
from associate import *
from evaluate_rpe import *
from evaluate_ate import *
from generate_pointc import *
from rclpy.node import Node
from slam_interfaces.msg import Posemsg


class PoseReceiver(Node):
    def __init__(self, pose_file):
        super().__init__("my_publisher")
        print("This is the pose receiver")
        self.mode = 0
        self.count = 0
        self.pose_file = pose_file
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
        with open(self.pose_file, "w") as file:
            for i, timestamp in enumerate(msg.timestamp):
                start_ind = counti
                end_ind = counti + 7
                # Convert timestamp to float and format it to 6 significant figures
                formatted_timestamp = f"{float(timestamp):.6f}"
                # Format each item in posequat to 9 decimal places
                formatted_posequat = " ".join(f"{x:.9f}" for x in msg.posequat[start_ind:end_ind])
                curr_msg = f"{formatted_timestamp} {formatted_posequat}\n"
                counti = counti + 7
                file.write(curr_msg)

class ReconstructPointCloud:
    def __init__(self, camera_traj, association_file, global_path,orb_camera,point_cloud_file):
        self.global_path = global_path
        self.association_file = association_file
        self.camera_traj = camera_traj
        self.orb_camera = orb_camera
        self.point_cloud_file = point_cloud_file
        self.rgb_files, self.depth_files, self.timestamps_rgb, self.timestamps_depth = [],[],[],[]
        if self.orb_camera:
            self.orb_camera_pop()
        else:
            print ("debugging rgbd files")
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

        write_ply(self.point_cloud_file,all_points)

    def orb_camera_pop(self):
        file_names_rgb = os.listdir("/home/anas/tmp_dataset/rgb")
        sorted_rgb_files = sorted(file_names_rgb)
        for file in sorted_rgb_files:
            self.timestamps_rgb.append(file[:-4])
            self.rgb_files.append("/home/anas/tmp_dataset/rgb/"+file)
            
        file_names_depth = os.listdir("/home/anas/tmp_dataset/depth")
        sorted_depth_files = sorted(file_names_depth)
        for file in sorted_depth_files:
            self.timestamps_depth.append(file[:-4])
            self.depth_files.append("/home/anas/tmp_dataset/depth/"+file)

    def populate_rgbd_files(self):
        with open(self.association_file, "r") as file:
            for line in file:
                contents = line.split(" ")
                self.timestamps_rgb.append(contents[0])
                self.rgb_files.append(self.global_path+"/"+contents[1])
                self.timestamps_depth.append(contents[2])
                self.depth_files.append(self.global_path+"/"+contents[3][:-1])


class FinalSteps:
    def __init__(self, predicted_path, dataset_path, associate_file,save_file):
        self.predicted_path = predicted_path
        self.dataset_path = dataset_path
        self.associate_file = associate_file
        self.save_file = save_file
        
        self.calculate_metrics()
        
    def calculate_metrics(self):
        """Calculate metrics"""

        args = {
            "first_file" : self.predicted_path,
            "second_file" : (self.dataset_path+"/groundtruth.txt"),
            "offset" : 0.0,
            "max_difference": 0.02,
            "scale": 1.0,
            "max_pairs": 10000,
            "fixed_delta": False,
            "delta": 1.0,
            "delta_unit": "s"
        }

        first_list = read_file_list(args["first_file"])
        second_list = read_file_list(args["second_file"])

        matches = associate(first_list, second_list,float(args["offset"]),float(args["max_difference"]))
        print (len(matches))    
        if len(matches)<2:
            sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")


        first_xyz = numpy.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
        second_xyz = numpy.matrix([[float(value)*float(args["scale"]) for value in second_list[b][0:3]] for a,b in matches]).transpose()
        rot,trans,trans_error = align(second_xyz,first_xyz)
        
        second_xyz_aligned = rot * second_xyz + trans
        
        first_stamps = list(first_list.keys())
        first_stamps.sort()
        first_xyz_full = numpy.matrix([[float(value) for value in first_list[b][0:3]] for b in first_stamps]).transpose()
        
        second_stamps = list(second_list.keys())
        second_stamps.sort()
        second_xyz_full = numpy.matrix([[float(value)*float(args["scale"]) for value in second_list[b][0:3]] for b in second_stamps]).transpose()
        second_xyz_full_aligned = rot * second_xyz_full + trans
        
        rmse = numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
        absolute_trans_error = numpy.mean(trans_error)
        
        traj_gt = read_trajectory(args["first_file"])
        traj_est = read_trajectory(args["second_file"])
        
        result = evaluate_trajectory(traj_gt,
                                    traj_est,
                                    int(args["max_pairs"]),
                                    args["fixed_delta"],
                                    float(args["delta"]),
                                    args["delta_unit"],
                                    float(args["offset"]),
                                    float(args["scale"]))
        
        stamps = numpy.array(result)[:,0]
        trans_error = numpy.array(result)[:,4]
        rot_error = numpy.array(result)[:,5]

        trans_rmse = numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
        rot_rmse = numpy.sqrt(numpy.dot(rot_error,rot_error) / len(rot_error)) * 180.0 / numpy.pi
        
        all_met = {
            "rmse": rmse,
            "absolute trans error": absolute_trans_error,
            "trans rmse": trans_rmse,
            "rot rmse": rot_rmse
        }
        with open(self.save_file+"/results.json", 'w') as json_file:
            json.dump(all_met, json_file, indent=4)        
        print (all_met)

if __name__=="__main__":
    obj = FinalSteps("/home/anas/ros2_ws/CameraTrajectory.txt","/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_xyz/","/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_xyz/associate.txt")
    
