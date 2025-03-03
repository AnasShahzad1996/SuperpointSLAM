import os
import cv2
import math
import time
import rclpy
import numpy as np
from PIL import Image
try:
    import torch
    import tensorflow as tf
except:
    print ("tensor flow not installed")

from utils import postProcessing
from rclpy.node import Node
from slam_interfaces.msg import Slamframe
from make_dataset import Camera_class


class superpoint_extractor(Node):
    def __init__(
        self, mode, initrinsic_file, extractor_type, association_file=None):
        super().__init__("my_publisher")
        self.mode = mode
        self.initrinsic_file = initrinsic_file
        self.extractor_type = extractor_type
        self.association_file = association_file
        #self.depth_scale = 5000
        self.depth_scale = 255
        self.timer_period = 0.1


        print(f"Using tensorflow {tf.__version__}") # make sure it's the nightly build
        os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
        self.interpreter = tf.compat.v1.lite.Interpreter(model_path="/home/anas/ros2_ws/src/superpoint_slam/models/my_lite.tflite")
        self.interpreter.allocate_tensors()
        
        self.config = {
        'superpoint': {
            'nms_radius': 4,
            'keypoint_threshold': 0.005,
            'max_keypoints': -1,
            'descriptor_dim': 256,
            'nms_radius': 4,
            'remove_borders': 4,        
            }
        }                
        #############################################################

        self.frame_id = 0
        self.send_int = 0
        self.wrapping_up_c = 0

        # if offline
        self.rgb_files = []
        self.depth_files = []
        self.rgb_timestamp = []
        self.depth_timestamp = []
        if self.mode == "offline":
            self.populate_rgbd_files()
            
        self.publisher_ = self.create_publisher(Slamframe, "keyframes", 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)



    def populate_rgbd_files(self):
        with open(self.association_file, "r") as file:
            for line in file:
                contents = line.split(" ")
                print(contents)
                self.rgb_files.append(contents[1])
                self.depth_files.append(contents[3][:-1])
                self.rgb_timestamp.append(contents[0])
                self.depth_timestamp.append(contents[2])

    def read_images(self):
        data_path = ("/").join(self.association_file.split("/")[:-1]) + "/"
        rgb_image = cv2.transpose(
            cv2.imread(data_path + self.rgb_files[self.frame_id], cv2.IMREAD_GRAYSCALE)
        ) 
        depth_image = cv2.imread(data_path + self.depth_files[self.frame_id], cv2.IMREAD_UNCHANGED)
        timestamp = self.rgb_timestamp[self.frame_id]
        return rgb_image, depth_image, timestamp

    def calculate_umax(self):
        HALF_PATCH_SIZE = 15
        # Initialize umax with the correct size
        umax = np.zeros(HALF_PATCH_SIZE + 1, dtype=int)

        vmax = int(math.floor(HALF_PATCH_SIZE * math.sqrt(2.0) / 2 + 1))
        vmin = int(math.ceil(HALF_PATCH_SIZE * math.sqrt(2.0) / 2))
        hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE

        for v in range(vmax + 1):
            umax[v] = int(round(math.sqrt(hp2 - v * v)))

        # Make sure we are symmetric
        v0 = 0
        for v in range(HALF_PATCH_SIZE, vmin - 1, -1):
            while umax[v0] == umax[v0 + 1]:
                v0 += 1
            umax[v] = v0
            v0 += 1            
        self.umax = umax

    def IC_Angle(self , image, pt):
        HALF_PATCH_SIZE = 15
        m_01 = 0
        m_10 = 0

        center_y, center_x = int(round(pt[1])), int(round(pt[0]))
        center = image[center_y, center_x]

        # Treat the center line differently, v=0
        for u in range(-HALF_PATCH_SIZE, HALF_PATCH_SIZE + 1):
            m_10 += u * int(image[center_y, center_x + u])

        # Go line by line in the circular patch
        step = image.strides[0]
        for v in range(1, HALF_PATCH_SIZE + 1):
            # Proceed over the two lines
            v_sum = 0
            d = self.umax[v]
            for u in range(-d, d + 1):
                val_plus = int(image[center_y + v, center_x + u])
                val_minus = int(image[center_y - v, center_x + u])
                v_sum += (val_plus - val_minus)
                m_10 += u * (val_plus + val_minus)
            m_01 += v * v_sum

        return cv2.fastAtan2(float(m_01), float(m_10))

    def orb_extractor(self, rgb_file=None, depth_file=None, timestamp=None):
        n_levels = 1  # Number of scale levels
        scale_factor = 1.2  # Scale factor between levels in the pyramid
        orb = cv2.ORB_create(scaleFactor=scale_factor, nlevels=n_levels)

        input_details = self.interpreter.get_input_details()
        output_details = self.interpreter.get_output_details()
        self.interpreter.set_tensor(input_details[0]['index'], np.float32((rgb_file/255).reshape((1,1,480,640))) )
        self.interpreter.invoke()
        mid1 = self.interpreter.get_tensor(output_details[1]['index'])
        des1 = self.interpreter.get_tensor(output_details[0]['index'])
        poster = postProcessing(self.config)
        [keypoints, scores, descriptors] = poster.process(mid1,des1)
        
        keypoints_cv2 = []
        self.calculate_umax()
        for x,y in keypoints[0][0:]:            
            if (float(y) > 620) or (float(y) < 40) or (float(x) > 440) or (float(x) < 40):
                pass
            else:             
                tmpc = cv2.KeyPoint(float(x), float(y), size=31,angle=self.IC_Angle(rgb_file,[float(x),float(y)]))
                keypoints_cv2.append(tmpc)
        keypoints, descriptors = orb.compute(rgb_file, tuple(keypoints_cv2))
        
        msg = Slamframe()
        if timestamp[-1] == ".":
            msg.timestamp = float(timestamp[:-1])
        else:
            msg.timestamp = float(timestamp)
        tmp_s = [1.0]
        msg.scalelevels = tmp_s
        msg.levels = 1
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
        orb = cv2.ORB_create(nfeatures=550, scaleFactor=1.2, nlevels=1)
        keypointsorb, descriptorsorb = orb.detectAndCompute(rgb_file, None)
        
        keypoints = keypoints + keypointsorb
        descriptors = np.vstack((descriptors, descriptorsorb))
        
        for i, currk in enumerate(keypoints):
            if (descriptors[i] is None):
                pass
            # Add the keypoints
            x, y = currk.pt
            msg.keypoints.append(y)
            msg.keypoints.append(x)

            depth_info = (depth_file[int(x), int(y)]) / self.depth_scale 
                               
            # kp.size, kp.angle, kp.response, kp.octave, kp.class_id
            msg.size.append(currk.size)
            msg.angle.append(currk.angle)
            msg.response.append(currk.response)
            msg.octave.append(currk.octave)
            #msg.octave.append(0.0)
            msg.class_id.append(currk.class_id)

            # Add descriptors
            msg.descriptors_size.append(len(descriptors[i]))
            tmp_des = tmp_des + list(descriptors[i].astype(float))
            
            tmp_depth.append(float(depth_info))
            point_num = point_num + 1
            
        msg.point_num = point_num
        msg.descriptors = tmp_des
        msg.depth = tmp_depth
        return msg

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
            if self.extractor_type == "superpoint":
                msg = self.orb_extractor(
                    rgb_file=rgb_image, depth_file=depth_image, timestamp=timestamp
                )

            self.publisher_.publish(msg)
            self.frame_id = self.frame_id + 1
            print("Publishing ", msg.id, "/",len(self.rgb_files))
        else:
            msg = self.wrapping_up()
            self.publisher_.publish(msg)
            if self.wrapping_up_c > 3:
                self.get_logger().info("Published all frames..")
                raise SystemExit


import os
import cv2
import rclpy
import numpy as np
from PIL import Image

from rclpy.node import Node
from slam_interfaces.msg import Slamframe

class FeatureExtractor(Node):
    def __init__(
        self, node_id, mode, initrinsic_file, extractor_type, association_file=None, dataset_online=None, time_interval=None
    ):
        super().__init__("my_publisher")
        self.mode = mode
        self.initrinsic_file = initrinsic_file
        self.extractor_type = extractor_type
        self.association_file = association_file
        self.depth_scale = 5000
        #self.depth_scale = 255
        self.timer_period = 0.1
        self.publisher_ = self.create_publisher(Slamframe, "keyframes", 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.frame_id = 0
        self.send_int = 0
        self.wrapping_up_c = 0

        # if offline
        self.rgb_files = []
        self.depth_files = []
        self.rgb_timestamp = []
        self.depth_timestamp = []
        if self.mode == "offline":
            self.populate_rgbd_files()
        elif self.mode == "online":
            self.start_time = time.time()
            self.camera = Camera_class(False,dataset_online)

    def populate_rgbd_files(self):
        with open(self.association_file, "r") as file:
            for line in file:
                contents = line.split(" ")
                print(contents)
                self.rgb_files.append(contents[1])
                self.depth_files.append(contents[3][:-1])
                self.rgb_timestamp.append(contents[0])
                self.depth_timestamp.append(contents[2])

    def read_images(self):
        data_path = ("/").join(self.association_file.split("/")[:-1]) + "/"
        rgb_image = cv2.transpose(
            cv2.imread(data_path + self.rgb_files[self.frame_id], cv2.IMREAD_GRAYSCALE)
        ) 
        depth_image = cv2.imread(data_path + self.depth_files[self.frame_id], cv2.IMREAD_UNCHANGED)
        timestamp = self.rgb_timestamp[self.frame_id]
        return rgb_image, depth_image, timestamp

    def orb_extractor(self, rgb_file=None, depth_file=None, timestamp=None):
        n_levels = 8  # Number of scale levels
        scale_factor = 1.2  # Scale factor between levels in the pyramid
        orb = cv2.ORB_create(nfeatures=1000, scaleFactor=1.2, nlevels=n_levels)

        keypoints, descriptors = orb.detectAndCompute(rgb_file, None)

        msg = Slamframe()
        if timestamp[-1] == ".":
            msg.timestamp = float(timestamp[:-1])
        else:
            msg.timestamp = float(timestamp)
        tmp_s = [1.0, 1.2, 1.44, 1.728, 2.0736, 2.48832, 2.98598, 3.58318]
        #tmp_s = [1.0]
        msg.scalelevels = tmp_s
        msg.levels = n_levels
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

        for i, currk in enumerate(keypoints):

            # Add the keypoints
            x, y = currk.pt
            msg.keypoints.append(y)
            msg.keypoints.append(x)

            depth_info = (depth_file[int(x), int(y)]) / self.depth_scale
            
            #print ("unique depth info : ",np.unique(depth_file))
            # kp.size, kp.angle, kp.response, kp.octave, kp.class_id
            msg.size.append(currk.size)
            msg.angle.append(currk.angle)
            msg.response.append(currk.response)
            msg.octave.append(currk.octave)
            msg.class_id.append(currk.class_id)

            # Add descriptors
            msg.descriptors_size.append(len(descriptors[i]))
            tmp_des = tmp_des + list(descriptors[i].astype(float))
            
            tmp_depth.append(float(depth_info))
            point_num = point_num + 1
        msg.point_num = point_num
        msg.descriptors = tmp_des
        msg.depth = tmp_depth
        return msg

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
        condition = self.frame_id < len(self.rgb_files)
        if self.mode == "online":
            condition = ( time.time() -self.start_time) < 90
        
        if self.send_int < 3:
            msg = self.populate_int()
            self.send_int = self.send_int + 1
            self.publisher_.publish(msg)
            #self.get_logger().info('Publishing: "%s"' % msg)
        elif condition:
            if self.mode == "offline":
                rgb_image, depth_image, timestamp = self.read_images()
            elif self.mode == "online":
                print ("Getting frame ...")
                rgb_image, depth_image, timestamp = self.camera.get_frame()
                                 
            if self.extractor_type == "orb":
                if rgb_image is not None:
                    msg = self.orb_extractor(
                        rgb_file=rgb_image, depth_file=depth_image, timestamp=timestamp
                    )

            if msg is not None:            
                self.publisher_.publish(msg)
                self.frame_id = self.frame_id + 1
                print("Publishing ", msg.id, "/",len(self.rgb_files))
                print("messages have the following length : ",len(msg.keypoints)/2)
        else:
            msg = self.wrapping_up()
            self.publisher_.publish(msg)
            if self.wrapping_up_c > 3:
                self.get_logger().info("Published all frames..")
                raise SystemExit
