#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "cone_finder_cpp/core_cone_finder.hpp"
#include <cv_bridge/cv_bridge.h>               // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp>                  // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include "common/common.hpp"
#include <cmath>
#include <boost/math/special_functions/round.hpp>
#include "cone_finder_cpp/tools.hpp"


int main(int argc, char* argv[])
{
  std::cout << "You have entered " << argc
         << " arguments:" << std::endl;

  std::string file_name;

  // Using a while loop to iterate through arguments
  int i = 0;
  while (i < argc) {
      std::cout << "oh " << i + 1 << ": " << argv[i]
            << std::endl;
      i++;
  }
  file_name = argv[1];
  // extarct data from file
  ToolsCam tools_{};              // tools cone finder node 
  Common common_{};              // tools cone finder node 
  std::cout << "oh" << std::endl;
  nav_msgs::msg::OccupancyGrid map{};
  geometry_msgs::msg::Pose bot_pose_{};
  cv::Mat in_image_;
  common_.DecodeYamlToMap(file_name+"_map.yaml",map);
  common_.DecodeYamlToPose(file_name+"_pose_cam.yaml",bot_pose_);
  common_.DecodeImage(file_name+"_img.jpg",in_image_);

  sensor_msgs::msg::CameraInfo camera_info_;
    camera_info_.height = in_image_.cols;
    camera_info_.width = in_image_.rows;
    camera_info_.distortion_model = "plumb_bob";
    camera_info_.d = {0.2, 0.3, 0.4, 0.5, 0.60};
    camera_info_.k= {
      591.574997, 0.000000, 334.168204 ,
      0.000000, 569.182214, 249.212991,
      0.000000, 0.000000 ,1.000000};

    camera_info_.p= {
      571.574997, 0.000000, 284.168204 ,0,
      0.000000, 569.182214, 249.212991 ,0,
      0.000000, 0.000000 ,1.000000 ,0};
    camera_info_.header.frame_id="Camera";

  CoreConeFinder core_;
  nav_msgs::msg::Odometry odom;
  odom.pose.pose = bot_pose_;
  core_.odom_ = {odom};
  // setup data
  
  int r_bot_{0};                  // row the base link of the robot
  int c_bot_{0};                  // column the base link of the robot
  core_.oc_ = {map};  
  std::tie(r_bot_, c_bot_) =  tools_.position_2_map(core_.odom_.pose_.position, core_.oc_.map_res_, core_.oc_.map_x0_, core_.oc_.map_y0_);

  int dev_mode_ = 10;

  geometry_msgs::msg::Point p_target_{};  
  vision_msgs::msg::BoundingBox2DArray BB_array;
  std::cout << "loaded complete" << std::endl;
  p_target_ = core_.update(in_image_, camera_info_, r_bot_, c_bot_, BB_array, dev_mode_);
  return 0;
}