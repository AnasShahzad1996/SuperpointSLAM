#!/usr/bin/env python
import numpy as np
import rclpy
from rclpy.node import Node
from slam_interfaces.msg import Slamframe
import os
import csv
from feature_extractor import FeatureExtractor
from pose_receiver import PoseReceiver, ReconstructPointCloud, FinalSteps
import dataset_config




def main(args=None):
    Node = 1
    print("### ORBextractor ###")

    orb_camera = False
    tmp = dataset_config.config9

    data_name = tmp["name"]
    camera_param = tmp["camera_param"]
    dataset_path = tmp["dataset_path"]
    dataset_config.make_associate_file(dataset_path)
    associate_file = tmp["associate_file"]
    
    # create result folder
    directory = "/home/anas/master_thesis/results/" + data_name 
    os.makedirs(directory, exist_ok=True)
    #######################

    pose_file = directory + "/CameraTrajectoryR.txt"
    ply_file = directory + "/reconstP.ply"
    
    rclpy.init(args=args)
    my_publisher = FeatureExtractor(
        Node,
        "offline",
        camera_param,
        "orb",
        associate_file,
    )
    try:
        rclpy.spin(my_publisher)
    except:
        print("Exiting node")
    my_publisher.destroy_node()
    rclpy.shutdown()

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

    reconstO = ReconstructPointCloud(pose_file, associate_file, dataset_path, orb_camera,ply_file)
    FinalSteps(pose_file,dataset_path,associate_file,directory)


if __name__ == "__main__":
    main()
