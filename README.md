# Superpoint_slam Package

This package implements a visual SLAM pipeline optimized for embedded devices using TPU acceleration, as detailed in the master's thesis "Optimizing Visual SLAM on Embedded Devices Using TPU Acceleration".  It leverages the SuperPoint network for feature extraction and a modified ORB-SLAM2 for backend processing.  This project specifically aims to accelerate visual SLAM on edge devices, replacing the CPU-intensive ORB feature computation with SuperPoint features suitable for Edge TPU acceleration.

## Overview

The core idea is to speed up the visual SLAM pipeline on edge devices.  Traditional ORB feature computation is CPU-intensive. This package explores replacing ORB features with SuperPoint features, which can be accelerated by an edge TPU. The project uses an Orbbec camera for RGB-D data and focuses on real-time camera pose estimation.  The pipeline is divided into a frontend and backend, with the frontend handling feature extraction (SuperPoint) and the backend performing optimization and mapping (based on ORB-SLAM2).  The project also explores image denoising techniques to improve SuperPoint detection accuracy.

## Dependencies

*   ROS 2 (Robot Operating System 2)
*   ORB-SLAM2 (A modified version adapted for this project)
*   SuperPoint Network implementation (TensorFlow Lite recommended for TPU compatibility)
*   Orbbec Camera ROS Driver
*   `slam_bk` package (assumed, based on the command provided; likely contains the modified ORB-SLAM2 backend)
*   TensorFlow Lite (for Edge TPU support)

## Installation

1.  **ROS 2 Installation:**  Follow the official ROS 2 installation instructions for your platform.

2.  **Clone the Repositories:** Clone this `superpoint_slam` package and the `slam_bk` package into your ROS 2 workspace.

    ```
    cd <your_ros2_workspace>/src
    ```

    Replace `<superpoint_slam_repository_url>` and `<slam_bk_repository_url>` with the actual URLs of the respective repositories.

3.  **Install Dependencies:** Use `rosdep` to install any missing dependencies.

    ```
    cd <your_ros2_workspace>
    rosdep install -i --from-path src --rosdistro <ros_distro> -y
    ```

    Replace `<ros_distro>` with your ROS 2 distribution name (e.g., `iron`, `humble`).

4.  **Build the Packages:** Build the packages using `colcon`.

    ```
    cd <your_ros2_workspace>
    colcon build
    . install/setup.bash
    ```

5. **Install TensorFlow Lite:** Install TensorFlow Lite with Edge TPU support following the official TensorFlow Lite documentation. This is crucial for SuperPoint acceleration.

## Usage

### 1. Running the ORB Feature Sender

This node is responsible for sending ORB features.  This might be used for comparison or as a fallback.


This command launches the `orb_sender.py` node within the `superpoint_slam` package.  It likely captures images, extracts ORB features, and publishes them as ROS 2 messages.

### 2. Running the SLAM Backend (slam_bk)

This node runs the SLAM backend, based on a modified ORB-SLAM2 implementation.


**Arguments:**

*   `rgbd_tum_keyframes`:  Specifies the mode of operation.  It appears to be configured to process RGB-D data using keyframes, likely from the TUM dataset.
*   `/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Vocabulary/ORBvoc.txt`:  Path to the ORB vocabulary file.  This file is essential for ORB-SLAM2's loop closure and relocalization.  **Important:**  Verify this path is correct for your system.

**Important Notes:**

*   **Dataset Path:**  The `slam_bk` node likely expects a specific directory structure for the TUM RGB-D dataset.  Ensure the dataset is correctly placed and that the node is configured to find the images and depth maps.  You might need to modify launch files or parameters to specify the correct paths.
*   **Vocabulary File:** The path to the vocabulary file is crucial. Double-check that the `ORBvoc.txt` file exists at the specified location.
*   **TPU Configuration:** To use the Edge TPU for SuperPoint acceleration, ensure the TensorFlow Lite runtime is correctly installed and configured.  You might need to set environment variables or modify the code to enable TPU inference.  See thesis sections 2.5 and 3.
*   **Orbbec Camera:** This project uses an Orbbec camera.  Make sure the Orbbec ROS driver is correctly installed and configured.
*   **Camera Calibration:** The `slam_bk` node requires camera calibration parameters.  Ensure the calibration parameters are accurate for your camera (see Section 2.4 of the thesis). The `superpoint_slam` may have internal calibration, check that it aligns with the other nodes.
*   **ROS Parameters:** Explore the available ROS parameters for both `orb_sender.py` and `slam_bk`.  These parameters allow you to fine-tune the behavior of the nodes.
*   **Image Denoising:** If you are using image denoising (Section 2.6), ensure that the denoising network is correctly integrated into the pipeline.

## Troubleshooting

*   **"Package not found" Error:**  Make sure the `superpoint_slam` and `slam_bk` packages are in your ROS 2 workspace and that you have sourced the `setup.bash` file after building.
*   **"Vocabulary file not found" Error:**  Double-check the path to the `ORBvoc.txt` file.
*   **TPU Issues:** Verify that the Edge TPU is correctly connected and that the TensorFlow Lite runtime is configured to use it.  Check the TensorFlow Lite documentation for troubleshooting steps.
*   **Poor Performance:** If you experience poor performance, try reducing the image resolution, adjusting the feature detection thresholds, optimizing the TensorFlow Lite model for the Edge TPU, or disabling image denoising.
*   **Camera Issues:** Make sure your Orbbec camera is connected and that the ROS driver is correctly configured. Check for proper udev rules if necessary.

## Further Information

Refer to the master's thesis "Optimizing Visual SLAM on Embedded Devices Using TPU Acceleration" by Anas Shahzad for a detailed explanation of the algorithms and implementation.  Pay close attention to the sections on:

*   **RGB-D SLAM (Section 1.3):** Provides background information on RGB-D SLAM.
*   **Edge RGB-D SLAM (Section 1.3.2):** Discusses the challenges and opportunities of running SLAM on edge devices.
*   **SuperPoint Network (Section 2.5):** Explains how the SuperPoint network is used for feature extraction.  Pay attention to the conversion to Tensorflow Lite.
*   **Image Denoising (Section 2.6):** Details the image denoising techniques used to improve SuperPoint accuracy.
*   **Camera Calibration (Section 2.4):** Details the camera calibration process.
*   **Results (Section 3):** Presents the experimental results and analysis.
