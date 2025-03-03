#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <curl/curl.h>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <fstream> // Include the <fstream> header
#include <sys/stat.h>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "libobsensor/h/Context.h"
#include "libobsensor/h/Device.h"
#include "libobsensor/h/Error.h"
#include "libobsensor/h/Filter.h"
#include "libobsensor/h/Frame.h"
#include "libobsensor/h/ObTypes.h"
#include "libobsensor/h/Pipeline.h"
#include "libobsensor/h/Sensor.h"
#include "libobsensor/h/StreamProfile.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "utils.hpp"  // local library
#include "ORBextractor.h"
#include "slam_interfaces/msg/slamframe.hpp" // Include your custom message header


void save_slam(slam_interfaces::msg::Slamframe curr_msg, std::string save_dist,cv::Mat colorMat, cv::Mat depthMat){
    std::cout << curr_msg.timestamp << std::endl;
    std::cout << curr_msg.mode << std::endl;

    std::string folderPath = save_dist + "/" + std::to_string(curr_msg.timestamp);
    int status = mkdir(folderPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    std::string rgb_filename = folderPath + "/rgb_image.jpeg";
    bool success = cv::imwrite(rgb_filename, colorMat);
    std::string depth_filename = folderPath + "/depth_image.jpeg";
    bool success1 = cv::imwrite(depth_filename, depthMat);
}


int stop;
void inthand(int signum) {
  if (signum == SIGINT) {
    stop = 0;
  }
}

class DepthPublisher : public rclcpp::Node {
 public:
  DepthPublisher(const std::string& settings_file)
      : Node("depth_publisher"), settings_file_(settings_file), count_(0) {
    this->stop_publishing = false;
    stop = 1;
    mode = 0;
    intrinsic_count = 0;
    this->publisher_ = this->create_publisher<slam_interfaces::msg::Slamframe>("keyframes", 10);

    std::signal(SIGINT, inthand);

    this->error_flag = false;
    ob_set_logger_severity(OB_LOG_SEVERITY_ERROR, &(this->error));
    this->check_error(this->error);
    this->pipeline = ob_create_pipeline(&(this->error));
    this->check_error(this->error);


    this->config = ob_create_config(&(this->error));
    this->check_error(this->error);
    this->depth_profile = NULL;
    this->profiles = ob_pipeline_get_stream_profile_list(
        this->pipeline, OB_SENSOR_DEPTH, &(this->error));
    this->check_error(this->error);

    if (this->profiles) {
      this->depth_profile = ob_stream_profile_list_get_video_stream_profile(this->profiles, 320, 0, OB_FORMAT_Y11, 30, &error);
    }
    ob_config_enable_stream(this->config, this->depth_profile, &(this->error));
    this->check_error(this->error);


    ob_config_set_align_mode(this->config, ALIGN_D2C_HW_MODE, &(this->error));
    this->check_error(this->error);

    this->color_profile = NULL;
    this->profiles = ob_pipeline_get_stream_profile_list(
        this->pipeline, OB_SENSOR_COLOR, &(this->error));

    if (this->error) {
      printf("Current device does not support color sensor!\n");
      ob_delete_error(this->error);
      this->error = NULL;
      ob_config_set_align_mode(this->config, ALIGN_DISABLE, &(this->error));
      this->check_error(this->error);
      return;
    }

    if (this->profiles) {
      this->color_profile =
          ob_stream_profile_list_get_profile(this->profiles, 0, &(this->error));
    }

    if (color_profile) {
      ob_config_enable_stream(this->config, this->color_profile,
                              &(this->error));
      this->check_error(this->error);
    }

    this->device = ob_pipeline_get_device(this->pipeline, &(this->error));
    this->check_error(this->error);

    ob_pipeline_start_with_config(this->pipeline, this->config, &(this->error));
    this->check_error(this->error);

    this->point_cloud = ob_create_pointcloud_filter(&(this->error));
    this->check_error(this->error);

    this->camera_param =
        ob_pipeline_get_camera_param(this->pipeline, &(this->error));
    this->check_error(this->error);
    ob_pointcloud_filter_set_camera_param(this->point_cloud, this->camera_param,
                                          &(this->error));




    // define the ORBextractor here
    cv::FileStorage fSettings(settings_file_, cv::FileStorage::READ);
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    ORBextractorLeft = new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    this->check_error(this->error);
    this->start_time = std::chrono::high_resolution_clock::now();
    if (this->error_flag) {
      return;
    } else {
      this->read_frames();
    }
  }

  // Shutting down all filters and channels
  void shutdown_camera() {
    ob_delete_filter(this->point_cloud, &(error));
    ob_pipeline_stop(this->pipeline, &(error));
    ob_delete_pipeline(this->pipeline, &(error));
    ob_delete_config(this->config, &(error));
    ob_delete_stream_profile(this->depth_profile, &(error));
    ob_delete_stream_profile(this->color_profile, &(error));
    ob_delete_stream_profile_list(this->profiles, &(error));
    rclcpp::shutdown();
  }

  // Orb objects
  std::string settings_file_;
  ORB_SLAM2::ORBextractor* ORBextractorLeft;

 private:
  // checking for errors
  void check_error(ob_error *error) {
    if (error) {
      printf("ob_error was raised: \n\tcall: %s(%s)\n",
             ob_error_function(error), ob_error_args(error));
      printf("\tmessage: %s\n", ob_error_message(error));
      printf("\terror type: %d\n", ob_error_exception_type(error));
      ob_delete_error(error);
      this->error_flag = true;
    }
  }

  // reading the frame
  bool read_frames() {
    while (stop == 1) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      if (intrinsic_count < 3){
        auto slam_msg = std::make_shared<slam_interfaces::msg::Slamframe>();

        std::string filename = "/home/anas/Desktop/code/VIO_slam/benchmarks/ORB_SLAM2/Examples/RGB-D/TUM1.yaml";
        std::ifstream file(filename);

        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string yamlString = buffer.str();
        slam_msg->mode = 0;
        slam_msg->camera_intrisic = yamlString;
        this->publisher_->publish(*slam_msg);

        std::cout << slam_msg->camera_intrisic << std::endl;

        intrinsic_count++;
        if (intrinsic_count >=3){
          mode = 1;
        }
      } else if(mode == 1){
        int count_ = 0;
        bool points_created = false;

        ob_frame *frameset = ob_pipeline_wait_for_frameset(this->pipeline, 10, &error);
        check_error(this->error);
        if(frameset == nullptr) {
            continue;
        }


        // my addition
        ob_frame *color_frame = ob_frameset_color_frame(frameset, &error);
        if (color_frame == nullptr){
          continue;
        }
        uint16_t *data1 = (uint16_t *)ob_frame_data(color_frame, &(this->error));
        int width = ob_video_frame_width(color_frame, &(this->error));
        int height = ob_video_frame_height(color_frame, &(this->error));
        check_error(this->error);

        auto frameType1 = ob_frame_get_type(color_frame, &error);
        auto format1 = ob_frame_format(color_frame, &error);

        cv::Mat rstMat1;
        cv::Mat rawMat1(height, width, CV_8UC2, data1);
        cv::cvtColor(rawMat1, rstMat1, cv::COLOR_YUV2BGR_UYVY);

        ob_frame *depth_frame = ob_frameset_depth_frame(frameset, &error);
        if (depth_frame == nullptr){
          continue;
        }

        void *data = ob_frame_data(depth_frame, &error);
        std::cout << "Height of the video : "<<height<<std::endl;
        std::cout << "Width of the video : "<<width<<std::endl;
        // Initializing depth variables
        cv::Mat rstMat;
        cv::Mat cvtMat;
        cv::Mat rawMat = cv::Mat(height, width, CV_16UC1, data);
        // depth frame pixel value multiply scale to get distance in millimeter
        float scale = ob_depth_frame_get_value_scale(depth_frame, &error);
        // threshold to 5.12m
        cv::threshold(rawMat, cvtMat, 5120.0f / scale, 0, cv::THRESH_TRUNC);
        cvtMat.convertTo(cvtMat, CV_8UC1, scale * 0.05);
        cv::applyColorMap(cvtMat, rstMat, cv::COLORMAP_JET);

        // rst mat1 has the color, rstmat has the depth
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        cv::Mat grayImage;
        cv::cvtColor(rstMat1, grayImage, cv::COLOR_BGR2GRAY);
        (*ORBextractorLeft)(grayImage,cv::Mat(),keypoints,descriptors);

        auto slam_msg = std::make_shared<slam_interfaces::msg::Slamframe>();
        slam_msg->id = 1;
        count_ = count_ + 1;

        std::vector<double> keyp;
        std::vector<double> keyang;
        std::vector<double> keysize;
        std::vector<double> keyresponse;
        std::vector<double> keyoctave;
        std::vector<double> keyclassid;

        std::vector<double> keydesc;
        std::vector<long unsigned int> desc_size;

        cv::Mat dImage;
        cv::cvtColor(rstMat, dImage, cv::COLOR_BGR2GRAY);
        std::vector<double> depth_vals;

        for (size_t i = 0; i < keypoints.size(); ++i) {
            keyp.push_back(keypoints[i].pt.x);
            keyp.push_back(keypoints[i].pt.y);
            keyang.push_back(keypoints[i].angle);
            keysize.push_back(keypoints[i].size);
            keyresponse.push_back(keypoints[i].response);
            keyoctave.push_back(keypoints[i].octave);
            keyclassid.push_back(keypoints[i].class_id);


            const auto& desc = descriptors.row(i);
            long unsigned int disc_col = desc.cols;
            desc_size.push_back(disc_col);
            // Write descriptor data
            for (int j = 0; j < desc.cols; ++j) {
                float curr_d_out = static_cast<float>(desc.at<unsigned char>(j));
                keydesc.push_back(curr_d_out);
            }

            // Access the pixel value at (x, y)
            uchar pixel_value = dImage.at<uchar>(keypoints[i].pt.x, keypoints[i].pt.y);
            float pixel_value_float = static_cast<float>(pixel_value);
            depth_vals.push_back(pixel_value_float);
        }

        auto start_time1 = std::chrono::high_resolution_clock::now();
        auto duration = start_time1.time_since_epoch();
        double start_time_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();

        slam_msg->timestamp = start_time_seconds;
        slam_msg->levels = 8;
        slam_msg->scalelevels = {1.0, 1.2, 1.44, 1.728, 2.0736, 2.48832, 2.98598, 3.58318};
        slam_msg->point_num = keypoints.size();



        // copy it to msg
        slam_msg->mode = 1;
        slam_msg->keypoints = keyp;
        slam_msg->descriptors = keydesc;
        slam_msg->size = keysize;
        slam_msg->angle = keyang;
        slam_msg->response = keyresponse;
        slam_msg->octave = keyoctave;
        slam_msg->class_id = keyclassid;
        slam_msg->descriptors_size = desc_size;
        slam_msg->depth = depth_vals;

        this->publisher_->publish(*slam_msg);

        std::string orbcam = "/home/anas/Desktop/code/VIO_slam/orbbec_camera/";
        save_slam(*slam_msg, orbcam, rstMat1, rstMat);
        ob_delete_frame(color_frame, &error);
        ob_delete_frame(depth_frame, &error);
        ob_delete_frame(frameset, &error);
      }else{
        std::cout << "Wrapping up" <<std::endl;
      }
    }

    this->shutdown_camera();

    return false;
  }


  // Depth cloud objects
  size_t count_;
  bool error_flag;
  bool stop_publishing;
  ob_error *error;
  ob_config *config;
  ob_device *device;
  ob_pipeline *pipeline;
  ob_filter *point_cloud;
  ob_camera_param camera_param;
  ob_stream_profile *depth_profile;
  ob_stream_profile *color_profile;
  ob_stream_profile_list *profiles;

  int mode;
  int intrinsic_count;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
  rclcpp::Publisher<slam_interfaces::msg::Slamframe>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <settings_file>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthPublisher>(argv[1]));
  return 0;
}
