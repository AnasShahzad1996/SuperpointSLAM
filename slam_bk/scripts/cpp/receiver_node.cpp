#include<opencv2/core/core.hpp>
#include <sys/stat.h>
#include<System.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "slam_interfaces/msg/slamframe.hpp" // Include your custom message header
#include "slam_interfaces/msg/posemsg.hpp"
#include<System.h>

using namespace slam_interfaces;

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(const std::string& vocab1, const std::string& camera_settings1)
  : Node("minimal_subscriber"), vocab_(vocab1), camera_settings_(camera_settings1)
  {
    subscription_ = this->create_subscription<slam_interfaces::msg::Slamframe>(    // CHANGE
      "keyframes", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    SLAM = new ORB_SLAM2::System(vocab_, ORB_SLAM2::System::RGBD);
    once_initial = 1;
    once_shutdown = 1;
  }

  void topic_callback(const slam_interfaces::msg::Slamframe & msg)
  {

      if (msg.mode == 0){

        if (once_initial==1){
        std::cout << "slam 1"<<std::endl;
        once_initial =2;
        std::cout << "Mode : "<<msg.mode << std::endl;
        std::ofstream tempFile("temp.yaml");
        tempFile << msg.camera_intrisic;
        tempFile.close();
        // Open the temporary YAML file using cv::FileStorage
        cv::FileStorage fsSettings("temp.yaml", cv::FileStorage::READ);
        SLAM->System::AddCamera(fsSettings, true);
        int result = std::remove("temp.yaml");
        }
      }else if (msg.mode == 1){
      int nlevel = msg.levels;
      double vTimestamps = msg.timestamp;
      vector <double> scalelevels1(msg.scalelevels);
      vector<double> depth_info(msg.depth);

      vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;

      int offset = 0;
      for (int i=0;i<msg.point_num;i++){

        cv::KeyPoint keypoint1(msg.keypoints[i*2], msg.keypoints[(i*2)+1], msg.size[i], msg.angle[i], msg.response[i], msg.octave[i], msg.class_id[i]);
        keypoints.push_back(keypoint1);

        std::vector<int> integers;
        for (int j=0;j<msg.descriptors_size[i];j++){
          integers.push_back(msg.descriptors[offset+j]);
        }
        cv::Mat row = cv::Mat(integers).reshape(1, 1);
        descriptors.push_back(row);

        offset = offset + msg.descriptors_size[i];
      }

      cv::Mat k_w = SLAM->TrackRGBDKeyPoint(keypoints, descriptors, depth_info, vTimestamps, scalelevels1, nlevel);
    }else{
      if (once_shutdown ==1){
      SLAM->Shutdown();
      SLAM->SaveTrajectoryTUM("CameraTrajectory.txt");
      once_shutdown = 2;
      rclcpp::shutdown();
      }
    }
  }

  int once_initial;
  int once_shutdown;
  ORB_SLAM2::System *SLAM;
  std::string vocab_;
  std::string camera_settings_;
  rclcpp::Subscription<slam_interfaces::msg::Slamframe>::SharedPtr subscription_;  // CHANGE
};

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() : Node("my_publisher"), count_(0) {
          std::cout << "Switching to sender"<<std::endl;
        publisher_ = this->create_publisher<slam_interfaces::msg::Posemsg>("pose", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MinimalPublisher::publish_message, this));

    }

private:
    void publish_message() {
      std::cout << "This is the message publisher "<<count_<<std::endl;
      if (count_ > 10){
        rclcpp::shutdown();
      }
      std::ifstream file("CameraTrajectory.txt");
      std::string line;
      std::vector<double> timestamp_numbers;
      std::vector<double> quat_numbers;
      while (std::getline(file, line)) {
          std::istringstream iss(line);
          double first_number;
          iss >> first_number;
          timestamp_numbers.push_back(first_number);

          double number;
          while (iss >> number) {
              quat_numbers.push_back(number);
          }
      }

      slam_interfaces::msg::Posemsg msg;
      msg.id        = count_;
      msg.timestamp = timestamp_numbers;
      msg.posequat  = quat_numbers;
      count_ = count_ + 1;
      publisher_->publish(msg);
    }

    rclcpp::Publisher<slam_interfaces::msg::Posemsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char **argv) {
    // Initialize the ROS node

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>(argv[1],argv[2]));
    rclcpp::shutdown();


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();

    return 0;
}
