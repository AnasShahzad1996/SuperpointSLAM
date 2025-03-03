#!/usr/bin/env python3
import os
import numpy as np
from pyorbbecsdk import Pipeline
from pyorbbecsdk import Config
from pyorbbecsdk import OBSensorType, OBFormat
from pyorbbecsdk import OBError
from pyorbbecsdk import *
import cv2
try:
    import open3d as o3d
except:
    print ("open3d not installed")
import numpy as np
import time
from utils import frame_to_bgr_image
import dataset_config
from PIL import Image

ESC_KEY = 27
PRINT_INTERVAL = 1  # seconds
MIN_DEPTH = 20  # 20mm
MAX_DEPTH = 10000  # 10000mm

class TemporalFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.previous_frame = None

    def process(self, frame):
        if self.previous_frame is None:
            result = frame
        else:
            result = cv2.addWeighted(frame, self.alpha, self.previous_frame, 1 - self.alpha, 0)
        self.previous_frame = result
        return result

class Camera_class:
    def __init__(self, display_image, dataset_path):
        self.pipeline = Pipeline()
        self.device = self.pipeline.get_device()
        self.device_info = self.device.get_device_info()
        self.device_pid = self.device_info.get_pid()
        self.config = Config()
        self.dataset_path = dataset_path
        self.display_image = display_image
        
        align_mode = 'HW'
        enable_sync = True
        try:
            self.profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            print ("this is not causing stuff 1")
            self.color_profile = self.profile_list.get_default_video_stream_profile()
            self.config.enable_stream(self.color_profile)
            print ("this is not causing stuff 2")
            self.profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            assert self.profile_list is not None
            self.depth_profile = self.profile_list.get_default_video_stream_profile()
            assert self.depth_profile is not None
            print ("this is not causing stuff 3")
            print("color profile : {}x{}@{}_{}".format(self.color_profile.get_width(),
                                                    self.color_profile.get_height(),
                                                    self.color_profile.get_fps(),
                                                    self.color_profile.get_format()))
            print("depth profile : {}x{}@{}_{}".format(self.depth_profile.get_width(),
                                                    self.depth_profile.get_height(),
                                                    self.depth_profile.get_fps(),
                                                    self.depth_profile.get_format()))
            print ("this is not causing stuff 4")
            self.config.enable_stream(self.depth_profile)
        except Exception as e:
            print(e)
            return
        if align_mode == 'HW':
            if self.device_pid == 0x066B:
                #Femto Mega does not support hardware D2C, and it is changed to software D2C
                self.config.set_align_mode(OBAlignMode.SW_MODE)
            else:
                self.config.set_align_mode(OBAlignMode.HW_MODE)
        elif align_mode == 'SW':
            self.config.set_align_mode(OBAlignMode.SW_MODE)
        else:
            self.config.set_align_mode(OBAlignMode.DISABLE)
        if enable_sync:
            try:
                self.pipeline.enable_frame_sync()
            except Exception as e:
                print(e)
        try:
            self.pipeline.start(self.config)
        except Exception as e:
            print(e)
        print ("this has done the pipeline")
        self.last_print_time = time.time()
        self.no_image = 0
        self.quit_key = True
        self.start_record = False
        if self.display_image:
            self.save_dataset()
                
    def save_dataset(self):
        while True:
            try:
                frames: FrameSet = self.pipeline.wait_for_frames(100)
                if frames is None:
                    continue
                color_frame = frames.get_color_frame()
                if color_frame is None:
                    continue
                # covert to RGB format
                color_image = frame_to_bgr_image(color_frame)
                if color_image is None:
                    print("failed to convert frame to image")
                    continue
                depth_frame = frames.get_depth_frame()
                if depth_frame is None:
                    continue

                width = depth_frame.get_width()
                height = depth_frame.get_height()
                scale = depth_frame.get_depth_scale()
                timestamp = time.time()
                                
                cropped_image = color_image[50:430, 50:590]
                resized_image_color = cv2.resize(cropped_image, (640, 480))  

                depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
                depth_data = depth_data.reshape((height, width))
                depth_data = depth_data.astype(np.float32)
                scale = depth_frame.get_depth_scale()
                depth_data = depth_data.astype(np.float32) * scale
                
                #print (np.unique(depth_data))
                
                depth_image = (depth_data * (255/5000)).astype(np.uint8) #cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                cropped_image = depth_image[50:430, 50:590]
                resized_image_depth = cv2.resize(cropped_image, (640, 480))

                format_timestamp = "{:.6f}".format(timestamp)
                file_name_depth = f"{self.dataset_path}/depth/{format_timestamp}.png"            
                file_name_col = f"{self.dataset_path}/rgb/{format_timestamp}.png"
                concatenated_image = np.hstack((cv2.cvtColor(resized_image_depth, cv2.COLOR_GRAY2BGR), resized_image_color))
                print (self.start_record,"|", (estimate_motion_blur(resized_image_color) > 20))

                if self.start_record :
                    Image.fromarray(resized_image_color).save(file_name_col, format='PNG', bits=16)
                    Image.fromarray(resized_image_depth).save(file_name_depth, format='PNG', bits=16)

                # Wait for key press
                key = cv2.waitKey(1) & 0xFF
                if key == ESC_KEY:  # ESC key to exit
                    print ("It started to record")
                    self.start_record = True
                elif key == ord('q'):
                    self.quit_key = False
                    break 

                ###############################
                # showing the frame image
                if self.display_image:                
                    variance_of_laplacian = estimate_motion_blur(resized_image_color)
                    text = f"Variance of Laplacian: {variance_of_laplacian:.2f}"
                    concatenated_image = np.hstack((cv2.cvtColor(resized_image_depth, cv2.COLOR_GRAY2BGR), resized_image_color))
                    cv2.putText(concatenated_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.imshow("SyncAlignViewer ", concatenated_image)
                    # showing the frame image
                ###############################
                
            except KeyboardInterrupt:
                cv2.destroyAllWindows()
                break
        self.pipeline.stop()
     
    def get_frame(self):
        try:
            print ("Getting frame...")
            color_img, depth_img = None, None
            frames: FrameSet = self.pipeline.wait_for_frames(100)
            if frames is None:
                return color_img, depth_img, None
            color_frame = frames.get_color_frame()
            if color_frame is None:
                return color_img, depth_img, None
            # covert to RGB format
            color_image = frame_to_bgr_image(color_frame)
            if color_image is None:
                return color_img, depth_img, None
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                return color_img, depth_img, None

            width = depth_frame.get_width()
            height = depth_frame.get_height()
            scale = depth_frame.get_depth_scale()
            timestamp = time.time()
                            
            cropped_image = color_image[50:430, 50:590]
            resized_image_color = cv2.resize(cropped_image, (640, 480))  
            format_timestamp = "{:.6f}".format(timestamp)

            file_name_col = f"{self.dataset_path}/rgb/{format_timestamp}.png"
            color_img = resized_image_color
            
            
            depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
            depth_data = depth_data.reshape((height, width))
            depth_data = depth_data.astype(np.float32)
            scale = depth_frame.get_depth_scale()
            depth_data = depth_data.astype(np.float32) * scale
            
            depth_image = (depth_data* (255/5000)).astype(np.uint8) #cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            cropped_image = depth_image[50:430, 50:590]
            resized_image_depth = cv2.resize(cropped_image, (640, 480))
            
            file_name_depth = f"{self.dataset_path}/depth/{format_timestamp}.png"            
            if estimate_motion_blur(resized_image_color) > 25:
                cv2.imwrite(file_name_col, resized_image_color)
                Image.fromarray(resized_image_depth).save(file_name_depth, format='PNG', bits=16)

            self.no_image = self.no_image + 1
            rgb_image_my = cv2.transpose(cv2.cvtColor(resized_image_color, cv2.COLOR_BGR2GRAY))
            return rgb_image_my, resized_image_depth, format_timestamp
        
        except:
            print ("Exception handling")
        return None, None, None

def estimate_motion_blur(image):
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply the Laplacian operator
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)

    # Calculate the variance of the Laplacian
    variance_of_laplacian = laplacian.var()

    return variance_of_laplacian

def display_images_with_blur_score(directory_path):
    # List all image files in the directory
    image_files = [f for f in os.listdir(directory_path) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff'))]
    
    if not image_files:
        print("No images found in the directory.")
        return
    
    index = 0
    while True:
        # Read the current image
        image_path = os.path.join(directory_path, image_files[index])
        image = cv2.imread(image_path)
        if image is None:
            print(f"Error: Unable to read the image {image_files[index]}.")
            return

        # Estimate the motion blur
        variance_of_laplacian = estimate_motion_blur(image)

        # Display the variance of Laplacian on the image
        text = f"Variance of Laplacian: {variance_of_laplacian:.2f}"
        cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show the image
        cv2.imshow("Image", image)

        # Wait for key press
        key = cv2.waitKey(0) & 0xFF
        
        if key == 27:  # ESC key to exit
            break
        elif key == 83 or key == 255:  # Right arrow key to move to the next image
            index = (index + 1) % len(image_files)
        elif key == 81:  # Left arrow key to move to the previous image
            index = (index - 1) % len(image_files)

    cv2.destroyAllWindows()

def create_point_cloud(rgb_image, depth_image, depth_scale=255, depth_trunc=100.0):
    h, w = depth_image.shape
    fx = fy = 525.0  # Focal length
    cx = w / 2
    cy = h / 2
    # Create meshgrid for pixel coordinates
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    u = u.astype(np.float32)
    v = v.astype(np.float32)
    # Calculate the 3D coordinates of each pixel
    z = (depth_image ) / 255
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    # Filter out points with invalid depth values
    valid = (z > (10/255)) & (z < depth_trunc)
    x = x[valid]
    y = y[valid]
    z = z[valid]
    colors = rgb_image[valid]
    # Create the point cloud
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(np.stack((x, y, z), axis=-1))
    point_cloud.colors = o3d.utility.Vector3dVector(colors / 255.0)
    
    cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=100, std_ratio=1.0)
    statistical_outlier_removed_pcd = point_cloud.select_by_index(ind)
    
    pt3d = np.asarray(statistical_outlier_removed_pcd.points)
    
    x_dim = pt3d[:,0]
    y_dim = pt3d[:,1]
    u_recog = ((x_dim * fx)/pt3d[:,2]) + cx
    u_recog = np.floor(u_recog).astype(int)
    y_recog = ((y_dim * fy)/pt3d[:,2]) + cy
    y_recog = np.floor(y_recog).astype(int)
    
    new_image = np.zeros((h,w))
    new_image[y_recog,u_recog] = pt3d[:,2]
    
    print("Point cloud after statistical outlier removal:")
    return statistical_outlier_removed_pcd, new_image

def save_remove_outliers(folder_path):
    file_names = []
    for root, dirs, files in os.walk(folder_path+"/depth/"):
        for file in files:
            file_names.append(file)

    for file in file_names:
        rgb_image = cv2.imread(folder_path+"/rgb/"+file)
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        # Load the depth image (16-bit grayscale)
        depth_image = cv2.imread(folder_path+"/depth/"+file, cv2.IMREAD_UNCHANGED)
        
        _, new_depth_img = create_point_cloud(rgb_image, depth_image)
        new_type = (new_depth_img*255).astype(np.uint8)
        
        Image.fromarray(new_type).save(folder_path+"/depth/"+file, format='PNG', bits=16)
            
    dataset_config.make_rgb_depth_file(folder_path)
    dataset_config.make_associate_file(folder_path)


if __name__=="__main__":
    print ("Start making the dataset...")
    # Path to the directory containing images
    #directory_path = '/home/anas/Desktop/datasets/slam_datasets/Custom_dataset/00_room_dataset/rgb'

    # Display images with blur score
    #display_images_with_blur_score(directory_path)
    Camera_class(True, "/home/anas/Desktop/datasets/slam_datasets/Custom_dataset/00_room_dataset")
    #Camera_class(True, "/home/anas/Desktop/datasets/slam_datasets/Custom_dataset/01_depth_wall")
    
    
    # swap new_depth and depth folders
    save_remove_outliers("/home/anas/Desktop/datasets/slam_datasets/Custom_dataset/00_room_dataset")
