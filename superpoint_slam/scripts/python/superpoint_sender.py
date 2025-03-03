#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from slam_interfaces.msg import Slamframe
import os
import csv
from feature_extractor import superpoint_extractor
from pose_receiver import PoseReceiver, ReconstructPointCloud, FinalSteps
import dataset_config

def main(args=None):
    Node = 1
    orb_camera = False
    print("### Superpoint extractor ###")


    tmp = dataset_config.config3

    data_name = tmp["name"]
    camera_param = tmp["camera_param"]
    dataset_path = tmp["dataset_path"]
    dataset_config.make_associate_file(dataset_path)
    associate_file = tmp["associate_file"]
    
    # create result folder
    directory = "/home/anas/master_thesis/results_super/" + data_name 
    os.makedirs(directory, exist_ok=True)
    #######################
    
    pose_file = directory + "/CameraTrajectoryR.txt"
    rclpy.init(args=args)
    my_publisher = superpoint_extractor(
        "offline",
        camera_param,
        "superpoint",
        associate_file,
    )

    try:
        rclpy.spin(my_publisher)
    except:
        print ("Exiting node")
    my_publisher.destroy_node()
    rclpy.shutdown()

    print("Switching to receiver")
    # switching to receiver
    rclpy.init(args=args)
    my_receiver = PoseReceiver(pose_file)
    try:
        rclpy.spin(my_receiver)
    except:
        print("Exiting pose receiver")
    my_receiver.destroy_node()
    
    rclpy.shutdown()     

    print("Creating a pointcloud")
    ply_file = directory + "/reconstP.ply"
    reconstO = ReconstructPointCloud(pose_file, associate_file, dataset_path, orb_camera, ply_file)
    FinalSteps(pose_file,dataset_path,associate_file,directory)


    
if __name__ == "__main__":
    main()
