# rgbd_dataset_freiburg1_room
config1 = {
    "name" : "rgbd_dataset_freiburg1_room",
    "camera_param" : "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM1.yaml",
    "associate_file" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_room/associate.txt",
    "dataset_path" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_room"        
}

# rgbd_dataset_freiburg1_xyz
config2 = {
    "name" : "rgbd_dataset_freiburg1_xyz",
    "camera_param" : "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM1.yaml",
    "associate_file" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_xyz/associate.txt",
    "dataset_path" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_xyz"        
}

# rgbd_dataset_freiburg2_desk
config3 = {
    "name" : "rgbd_dataset_freiburg2_desk",
    "camera_param" : "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM2.yaml",
    "associate_file" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg2_desk/associate.txt",
    "dataset_path" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg2_desk"        
}

# rgbd_dataset_freiburg2_pioneer_360
config4 = {
    "name" : "rgbd_dataset_freiburg2_pioneer_360",
    "camera_param" : "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM2.yaml",
    "associate_file" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg2_pioneer_360/associate.txt",
    "dataset_path" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg2_pioneer_360"        
}

# rgbd_dataset_freiburg1_desk   
config5 = {
    "name" : "rgbd_dataset_freiburg1_desk",
    "camera_param" : "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM1.yaml",
    "associate_file" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_desk/associate.txt",
    "dataset_path" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_desk"        
}

# rgbd_dataset_freiburg2_xyz
config6 = {
    "name" : "rgbd_dataset_freiburg2_xyz",
    "camera_param" : "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM2.yaml",
    "associate_file" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg2_xyz/associate.txt",
    "dataset_path" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg2_xyz"        
}

# rgbd_dataset_freiburg1_desk2  
config7 = {
    "name" : "rgbd_dataset_freiburg1_desk2",
    "camera_param" : "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM1.yaml",
    "associate_file" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_desk2/associate.txt",
    "dataset_path" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_desk2"        
}

# rgbd_dataset_freiburg3_long_office_household
config8 = {
    "name" : "rgbd_dataset_freiburg3_long_office_household",
    "camera_param" : "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM3.yaml",
    "associate_file" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg3_long_office_household/associate.txt",
    "dataset_path" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg3_long_office_household"        
}

# rgbd_dataset_freiburg1_plant  
config9 = {
    "name" : "rgbd_dataset_freiburg1_plant",
    "camera_param" : "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM1.yaml",
    "associate_file" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_plant/associate.txt",
    "dataset_path" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg1_plant"        
}

# rgbd_dataset_freiburg3_nostructure_notexture_far
config10 = {
    "name" : "rgbd_dataset_freiburg3_nostructure_notexture_far",
    "camera_param" : "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM3.yaml",
    "associate_file" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg3_nostructure_notexture_far/associate.txt",
    "dataset_path" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg3_nostructure_notexture_far"        
}

config11 = {
    "name" : "rgbd_dataset_freiburg3_teddy",
    "camera_param" : "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM3.yaml",
    "associate_file" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg3_teddy/associate.txt",
    "dataset_path" : "/home/anas/Desktop/datasets/slam_datasets/rgbd_dataset_freiburg3_teddy"        
}

config12 = {
    "name" : "00_room_chair",
    "camera_param" : "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM3.yaml",
    "associate_file" : "/home/anas/Desktop/datasets/slam_datasets/Custom_dataset/00_room_dataset/associate.txt",
    "dataset_path" : "/home/anas/Desktop/datasets/slam_datasets/Custom_dataset/00_room_dataset"        
}


from associate import *
from evaluate_rpe import *

def make_associate_file(dataset_path):
    rgb_list = read_file_list(dataset_path+"/rgb.txt")
    depth_list = read_file_list(dataset_path+"/depth.txt")

    matches_rgb_depth = associate(rgb_list, depth_list,0.0,0.02)  
    matches_rgb_depth.sort()

    with open(dataset_path+'/associate.txt', 'w') as file:
        # Write lines to the file
        for i, tuple in enumerate(matches_rgb_depth):
            
            rgb_d = "{:.6f}".format(tuple[0])
            depth_d = "{:.6f}".format(tuple[1])
            line = str(rgb_d) + " "+"rgb/"+ str(rgb_d)+".png "
            line1 = str(depth_d) + " "+"depth/"+ str(depth_d)+".png\n"
            
            final_line = line + line1
            file.write(final_line)

def make_rgb_depth_file(folder_path):
    file_names = []
    for root, dirs, files in os.walk(folder_path+"/depth/"):
        for file in files:
            file_names.append(file)
            
    with open(folder_path+'/depth.txt', 'w') as file:
        # Write lines to the file
        for curr_file in file_names:
            line = (".").join(curr_file.split(".")[0:2]) + " depth/" +curr_file+"\n"
            file.write(line) 

    with open(folder_path+'/rgb.txt', 'w') as file:
        # Write lines to the file
        for curr_file in file_names:
            line = (".").join(curr_file.split(".")[0:2]) + " rgb/" +curr_file+"\n"
            file.write(line)