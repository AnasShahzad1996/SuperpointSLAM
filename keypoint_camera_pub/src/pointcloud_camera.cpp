#include <libobsensor/h/Context.h>
#include <libobsensor/h/Device.h>
#include <libobsensor/h/Error.h>
#include <libobsensor/h/Filter.h>
#include <libobsensor/h/Frame.h>
#include <libobsensor/h/ObTypes.h>
#include <libobsensor/h/Pipeline.h>
#include <libobsensor/h/Sensor.h>
#include <libobsensor/h/StreamProfile.h>
#include <stdio.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

#include "utils.hpp"  // local library

int stop;
void inthand(int signum) {
  if (signum == SIGINT) {
    stop = 0;
  }
}

class PointCloudPublisher : public rclcpp::Node {
 public:
  PointCloudPublisher() : Node("depth_publisher"), count_(0) {
    this->stop_publishing = false;
    stop = 1;
    this->publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "olive_pointcloud", 10);
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
      this->depth_profile =
          ob_stream_profile_list_get_profile(this->profiles, 0, &(this->error));
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
      int count = 0;
      bool points_created = false;

      ob_frame *frameset =
          ob_pipeline_wait_for_frameset(this->pipeline, 10, &error);
      this->check_error(this->error);
      if (frameset != NULL) {
        ob_pointcloud_filter_set_point_format(point_cloud, OB_FORMAT_RGB_POINT,
                                              &error);
        this->check_error(this->error);
        ob_frame *pointsFrame =
            ob_filter_process(point_cloud, frameset, &(this->error));
        this->check_error(this->error);
        if (pointsFrame != NULL) {

          count_ ++;
          this->end_time = std::chrono::high_resolution_clock::now();
          int duration = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count());
          if (duration > 30){
            this->start_time = this->end_time;
            std::cout << "The number of messages published per second :"<<(count_/duration)<<std::endl;
            count_ = 0;
          }

          this->publish_pointcloud(pointsFrame);
          ob_delete_frame(pointsFrame, &error);
          this->check_error(this->error);
        }
        ob_delete_frame(frameset, &(this->error));
        this->check_error(this->error);
        if (pointsFrame != NULL){
          ob_delete_frame(pointsFrame, &(this->error));
        }
      }
    }

    this->shutdown_camera();
    return false;
  }

  // Publishing point clouds
  void publish_pointcloud(ob_frame *frame) {
    sensor_msgs::msg::PointCloud2 pc2;
    int pointsSize = ob_frame_data_size(frame, &error) / sizeof(ob_color_point);
    check_error(error);

    // Populate header
    pc2.header.stamp = rclcpp::Clock().now();
    pc2.header.frame_id = "map";  // Replace with your desired frame ID

    pc2.height = 1;
    pc2.width = pointsSize;

    pc2.fields.resize(3);  // x, y, z
    pc2.fields[0].name = "x";
    pc2.fields[0].offset = 0;
    pc2.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2.fields[0].count = 1;

    pc2.fields[1].name = "y";
    pc2.fields[1].offset = 4;
    pc2.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2.fields[1].count = 1;

    pc2.fields[2].name = "z";
    pc2.fields[2].offset = 8;
    pc2.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2.fields[2].count = 1;

    pc2.is_bigendian = false;
    pc2.point_step = sizeof(float) * 3;  // Size of each point
    pc2.row_step = pc2.point_step * pc2.width;
    pc2.is_dense = true;

    pc2.data.resize(pc2.row_step * pc2.height);

    // Fill in points to form a line
    ob_color_point *point =
        (ob_color_point *)ob_frame_data(frame, &(this->error));
    this->check_error(this->error);
    for (int i = 0; i < pointsSize; i++) {
      float *point_data =
          reinterpret_cast<float *>(pc2.data.data() + i * pc2.point_step);
      point_data[0] = point->x / 1000;  // x-coordinate
      point_data[1] = point->y / 1000;
      ;  // y-coordinate (for simplicity, set to zero)
      point_data[2] = point->z / 1000;
      ;  // z-coordinate (for simplicity, set to zero)
      point++;
    }

    this->publisher_->publish(pc2);
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

  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPublisher>());
  return 0;
}
