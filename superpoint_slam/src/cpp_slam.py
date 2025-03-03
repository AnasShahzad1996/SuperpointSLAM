#!/usr/bin/env python
import os
import cv2
import csv
import yaml
import time
import rclpy
import numpy as np
import dataset_config

from PIL import Image
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

def make_png_files(dataset_folder):
   timestamp = time.time()
   for filename in os.listdir(dataset_folder+"/depth"):
        if filename.endswith('.yml'):
            file_path = os.path.join(dataset_folder+"/depth", filename)
            
            # Read the YML file
            fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
            matrix = fs.getNode('matrix').mat()
            fs.release()
            
            # Normalize the matrix to 0-255 range for image
            depth_image = (matrix).astype(np.uint8)
            
            # Create an image from the matrix
            image = Image.fromarray(depth_image)
            
            # Save the image as PNG
            counti = float(os.path.splitext(filename)[0])
            new_filename = format_timestamp = "{:.6f}".format(timestamp+counti) + '.png'
            new_file_path = os.path.join(dataset_folder+"/depth", new_filename)
            image.save(new_file_path)
            
            print(f"Saved {new_filename}")
            
            # Remove the original YML file
            os.remove(file_path)
            print(f"Removed {filename}")
            
   for filename in os.listdir(dataset_folder+"/rgb"):
        if filename.endswith('.yml'):
            file_path = os.path.join(dataset_folder+"/rgb", filename)
            
            # Read the YML file
            # Read the YML file
            fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
            matrix = fs.getNode('matrix').mat()
            fs.release()
            
            # Normalize the matrix to 0-255 range for image
            depth_image = (matrix).astype(np.uint8)
            
            # Create an image from the matrix
            image = Image.fromarray(depth_image)
            
            # Save the image as PNG
            counti = float(os.path.splitext(filename)[0])
            new_filename = format_timestamp = "{:.6f}".format(timestamp+counti) + '.png'
            new_file_path = os.path.join(dataset_folder+"/rgb", new_filename)
            image.save(new_file_path)
            
            print(f"Saved {new_filename}")
            
            # Remove the original YML file
            os.remove(file_path)
            print(f"Removed {filename}")

def main(args=None):

    orb_camera = False
    main_dataset = "/home/olive/custom_dataset"
    

    
    config_off = {
        "name" : "slam_camera",
        "camera_param" : "/home/olive/TUM1.yaml",
        "associate_file" : f"{main_dataset}/associate.txt",
        "dataset_path" : f"{main_dataset}"        
    }
    make_png_files(config_off["dataset_path"])
    
    ###### Saving creating dataset ######
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
