#!/usr/bin/env python
import os
import csv
import time
import rclpy
import numpy as np
import dataset_config

from rclpy.node import Node
from slam_interfaces.msg import Slamframe
from feature_extractor import FeatureExtractor
from make_dataset import save_remove_outliers, Camera_class
from pose_receiver import PoseReceiver, ReconstructPointCloud, FinalSteps


def create_dataset(dataset_file):
    os.makedirs(dataset_file, exist_ok=True)
    os.makedirs(dataset_file+"/rgb", exist_ok=True)
    os.makedirs(dataset_file+"/depth", exist_ok=True)
    return False
        
def main(args=None):

    orb_camera = False
    timestamp = time.time()
    format_timestamp = "{:.6f}".format(timestamp)
    
    config_off = {
        "name" : f"{format_timestamp}",
        "camera_param" : "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM3.yaml",
        "associate_file" : f"/home/anas/Desktop/datasets/slam_datasets/Custom_dataset/{format_timestamp}/associate.txt",
        "dataset_path" : f"/home/anas/Desktop/datasets/slam_datasets/Custom_dataset/{format_timestamp}"        
    }
    
    ###### Saving creating dataset ######
    create_dataset(config_off["dataset_path"])
    Camera_class(True, config_off["dataset_path"])
    #save_remove_outliers(config_off["dataset_path"])
    dataset_config.make_rgb_depth_file(config_off["dataset_path"])
    dataset_config.make_associate_file(config_off["dataset_path"])
    #####################################
    
    pose_file = config_off["dataset_path"] + "/CameraTrajectoryR.txt"
    ply_file = config_off["dataset_path"] + "/reconstP.ply"
    
    rclpy.init(args=args)
    my_publisher = FeatureExtractor(
        Node,
        "offline",
        config_off["camera_param"],
        "orb",
        config_off["associate_file"],
    )
    try:
        rclpy.spin(my_publisher)
    except:
        print("Exiting node")
    my_publisher.destroy_node()
    rclpy.shutdown()
    
    #######################################
    
    print("Switching to receiver")
    # switching to receiver
    rclpy.init(args=args)
    my_receiver = PoseReceiver(pose_file)
    try:
        rclpy.spin(my_receiver)
    except:
        print ("Exiting receiver node")
    my_receiver.destroy_node()
    
    rclpy.shutdown()
    print("Creating a pointcloud")

    reconstO = ReconstructPointCloud(pose_file, config_off["associate_file"], config_off["dataset_path"], orb_camera, ply_file)


if __name__=="__main__":
    print ("Offline slam starts...")
    main()