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
#include <curl/curl.h>
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

class IRPublisher : public rclcpp::Node {
 public:
  IRPublisher() : Node("IR_publisher"), count_(0) {
    // Get all stream profiles of the infrared camera, including stream
    // resolution, frame rate, and frame format
    this->stream_available = 0;
    this->pipe = ob_create_pipeline(&(this->error));
    check_error(this->error);
    this->config = ob_create_config(&(this->error));
    check_error(this->error);
    this->ir_profile = NULL;
    this->profiles = ob_pipeline_get_stream_profile_list(
        this->pipe, OB_SENSOR_IR, &(this->error));
    check_error(this->error);

    this->ir_profile = ob_stream_profile_list_get_video_stream_profile(
        profiles, 640, 0, OB_FORMAT_Y16, 30, &(this->error));
    if (this->error) {
      // stream_available = 1;
      this->ir_profile =
          ob_stream_profile_list_get_profile(this->profiles, 0, &(this->error));
      ob_delete_error(this->error);
      this->error = nullptr;
    }

    if (stream_available == 0) {
      ob_config_enable_stream(this->config, this->ir_profile, &(this->error));
      check_error(this->error);
      this->device = ob_pipeline_get_device(this->pipe, &(this->error));
      check_error(this->error);

      // Determine whether to support switching left and right ir channels
      if (ob_device_is_property_supported(
              this->device, OB_PROP_IR_CHANNEL_DATA_SOURCE_INT,
              OB_PERMISSION_READ_WRITE, &(this->error))) {
        // Gemini2 products support SENSOR_IR to select sensor output, 0 is left
        // IR, 1 is right IR.
        int32_t dataSource = 0;
        ob_device_set_int_property(this->device,
                                   OB_PROP_IR_CHANNEL_DATA_SOURCE_INT,
                                   dataSource, &(this->error));
        check_error(this->error);
      }
      check_error(this->error);
      ob_pipeline_start_with_config(this->pipe, this->config, &(this->error));
      check_error(this->error);

      this->width =
          ob_video_stream_profile_width(this->ir_profile, &(this->error));
      check_error(this->error);
      this->height =
          ob_video_stream_profile_height(this->ir_profile, &(this->error));
      check_error(this->error);
    }
    cv_bridge_ = std::make_shared<cv_bridge::CvImage>();
    this->node = rclcpp::Node::make_shared("olive_IRImageNode", this->options);
    image_transport::ImageTransport it(this->node);
    this->pub = it.advertise("olive_IRImage", 1);
    std::signal(SIGINT, inthand);
    stop = 1;
    this->start_time = std::chrono::high_resolution_clock::now();
    if (this->stream_available == 0) {
      publish_frames();
    }
  }

  void shutdown_camera() {
    ob_pipeline_stop(this->pipe, &(this->error));
    check_error(this->error);

    ob_delete_stream_profile(this->ir_profile, &(this->error));
    check_error(this->error);

    ob_delete_stream_profile_list(this->profiles, &(this->error));
    check_error(this->error);

    ob_delete_device(this->device, &(this->error));
    check_error(this->error);

    ob_delete_pipeline(this->pipe, &(this->error));
    check_error(this->error);

    rclcpp::shutdown();
  }

  int stream_available;

 private:
  void publish_frames() {
    while (stop == 1) {
      ob_frame *frameset =
          ob_pipeline_wait_for_frameset(this->pipe, 10, &(this->error));
      check_error(this->error);

      if (frameset == nullptr) {
        continue;
      }
      ob_frame *ir_frame = ob_frameset_ir_frame(frameset, &(this->error));
      check_error(this->error);

      // Getting the ir frame and converting it into cv matrix
      void *data = ob_frame_data(ir_frame, &(this->error));
      ob_frame_type frameType = ob_frame_get_type(ir_frame, &(this->error));
      ob_format format = ob_frame_format(ir_frame, &(this->error));
      cv::Mat cvtMat;
      cv::Mat rstMat;
      cv::Mat rawMat = cv::Mat(this->height, this->width, CV_16UC1, data);
      rawMat.convertTo(cvtMat, CV_8UC1, 1.0 / 16.0f);
      cv::cvtColor(cvtMat, rstMat, cv::COLOR_GRAY2RGB);
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

      ob_delete_frame(frameset, &error);
      check_error(this->error);
    }

    shutdown_camera();
  }

  int count_;
  uint32_t width;
  uint32_t height;
  ob_error *error;
  ob_pipeline *pipe;
  ob_device *device;
  ob_config *config;
  ob_stream_profile *ir_profile;
  ob_stream_profile_list *profiles;
  rclcpp::NodeOptions options;
  std::shared_ptr<cv_bridge::CvImage> cv_bridge_;
  rclcpp::Node::SharedPtr node;
  image_transport::Publisher pub;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
};

// Main function
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IRPublisher>());

  return 0;
}
