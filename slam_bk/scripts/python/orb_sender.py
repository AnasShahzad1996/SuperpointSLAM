#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from slam_interfaces.msg import Slamframe
import os
import csv
from Feature_Extractor import FeatureExtractor
from Pose_Receiver import PoseReceiver, ReconstructPointCloud

def main(args=None):
    Node = 1
    print("### ORBextractor ###")

    camera_param = "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM1.yaml"
    associate_file = "/home/olive/long_orb/associate.txt"
    dataset_path = (
        "/home/olive/long_orb"
    )
    
    associate_file = "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_xyz/associate.txt"
    dataset_path = (
        "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_xyz/"
    )
    
    associate_file = "/home/anas/Desktop/datasets/slam_datasets/Custom_dataset/00_room_dataset/associate.txt"
    dataset_path = ("/home/anas/Desktop/datasets/slam_datasets/Custom_dataset/00_room_dataset")    

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
    pose_file = "receivedpose.txt"
    rclpy.init(args=args)
    my_receiver = PoseReceiver(pose_file, dataset_path, associate_file)
    try:
        rclpy.spin(my_receiver)
    except:
        print("Exiting pose receiver")
    my_receiver.destroy_node()
    rclpy.shutdown()
    print("Creating a pointcloud")
    reconstO = ReconstructPointCloud("CameraTrajectoryR.txt", associate_file, dataset_path)

if __name__ == "__main__":
    main()
