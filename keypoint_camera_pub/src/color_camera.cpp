#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

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
#include "sensor_msgs/msg/image.hpp"
#include "utils.hpp"  // local library

int stop;
void inthand(int signum) {
  if (signum == SIGINT) {
    stop = 0;
  }
}

void check_error(ob_error *error) {
  if (error) {
    printf("ob_error was raised: \n\tcall: %s(%s)\n", ob_error_function(error),
           ob_error_args(error));
    printf("\tmessage: %s\n", ob_error_message(error));
    printf("\terror type: %d\n", ob_error_exception_type(error));
    ob_delete_error(error);
    exit(EXIT_FAILURE);
  }
}

class CameraPublisher : public rclcpp::Node {
 public:
  CameraPublisher() : Node("camera_publisher"), count_(0) {
    cv_bridge_ = std::make_shared<cv_bridge::CvImage>();
    this->node = rclcpp::Node::make_shared("olive_RGBImageNode", this->options);
    image_transport::ImageTransport it(this->node);
    this->pub = it.advertise("olive_RGBImage", 1);

    this->pipe = ob_create_pipeline(&(this->error));
    check_error(this->error);

    this->config = ob_create_config(&(this->error));
    check_error(this->error);

    // Get color profiles
    this->color_profile = nullptr;
    this->profiles = ob_pipeline_get_stream_profile_list(
        this->pipe, OB_SENSOR_COLOR, &(this->error));

    if (this->error) {
      printf("Current device is not support color sensor!\n");
      exit(EXIT_FAILURE);
    }

    // Find the corresponding profile according to specied format and choose
    // RGB888 format
    this->color_profile = ob_stream_profile_list_get_video_stream_profile(
        this->profiles, 1280, 0, OB_FORMAT_UYVY, 30, &(this->error));
    if (this->error) {
      this->color_profile =
          ob_stream_profile_list_get_profile(this->profiles, 0, &(this->error));
      ob_delete_error(this->error);
      this->error = nullptr;
    }

    // Enable stream
    ob_config_enable_stream(this->config, this->color_profile, &(this->error));
    check_error(this->error);

    // Get Device through pipeline
    this->device = ob_pipeline_get_device(this->pipe, &(this->error));
    check_error(this->error);

    ob_pipeline_start_with_config(this->pipe, this->config, &(this->error));
    check_error(this->error);

    std::signal(SIGINT, inthand);
    stop = 1;
    this->start_time = std::chrono::high_resolution_clock::now();
    publish_camera_image();
  }

  // Shutdown camera
  void shutdown_camera() {
    ob_pipeline_stop(this->pipe, &(this->error));
    check_error(this->error);

    ob_delete_stream_profile(this->color_profile, &(this->error));
    check_error(this->error);

    ob_delete_stream_profile_list(this->profiles, &(this->error));
    check_error(this->error);

    ob_delete_device(this->device, &(this->error));
    check_error(this->error);

    ob_delete_pipeline(this->pipe, &(this->error));
    check_error(this->error);
    rclcpp::shutdown();
  }

 private:
  void publish_camera_image() {
    while (stop == 1) {
      // Wait for up to 100ms for a frameset in blocking mode.
      ob_frame *frameset = ob_pipeline_wait_for_frameset(pipe, 10, &error);
      check_error(this->error);

      if (frameset == nullptr) {
        continue;
      }

      ob_frame *color_frame = ob_frameset_color_frame(frameset, &error);
      check_error(this->error);
      if (color_frame != nullptr) {
        int width = ob_video_frame_width(color_frame, &(this->error));
        int height = ob_video_frame_height(color_frame, &(this->error));
        check_error(this->error);

        uint16_t *data = (uint16_t *)ob_frame_data(color_frame, &(this->error));
        check_error(this->error);

        auto frameType = ob_frame_get_type(color_frame, &error);
        auto format = ob_frame_format(color_frame, &error);

        cv::Mat rstMat;
        cv::Mat rawMat(height, width, CV_8UC2, data);
        cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_UYVY);
        std_msgs::msg::Header hdr;
        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(hdr, "bgr8", rstMat).toImageMsg();

        count_ ++;
        this->end_time = std::chrono::high_resolution_clock::now();
        int duration = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count());
        if (duration > 30){
          this->start_time = this->end_time;
          std::cout << "The number of messages published per second :"<<(count_/duration)<<std::endl;
          count_ = 0;
        }
        pub.publish(msg);
      }
      ob_delete_frame(color_frame, &error);
      ob_delete_frame(frameset, &error);
      //check_error(this->error);
    }
    this->shutdown_camera();
  }

  int count_;
  rclcpp::NodeOptions options;
  std::shared_ptr<cv_bridge::CvImage> cv_bridge_;
  rclcpp::Node::SharedPtr node;
  image_transport::Publisher pub;
  ob_device *device;
  ob_pipeline *pipe;
  ob_error *error;
  ob_config *config;
  ob_stream_profile *color_profile;
  ob_stream_profile_list *profiles;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  return 0;
}
