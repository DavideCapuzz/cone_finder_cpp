#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "cone_finder_cpp/search_target.hpp"
#include <cv_bridge/cv_bridge.h>               // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp>                  // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include "common/common.hpp"
#include <cmath>
#include <boost/math/special_functions/round.hpp>
#include "cone_finder_cpp/tools.hpp"

int main(int argc, char *argv[])
{
  std::cout << "You have entered " << argc
            << " arguments:" << std::endl;

  std::string file_name;

  // Using a while loop to iterate through arguments
  int i = 0;
  while (i < argc)
  {
    std::cout << "oh " << i + 1 << ": " << argv[i]
              << std::endl;
    i++;
  }
  file_name = argv[1];
  // extarct data from file
  ToolsCam tools_{}; // tools cone finder node
  Common common_{};  // tools cone finder node
  std::cout << "oh" << std::endl;
  nav_msgs::msg::OccupancyGrid map{};
  geometry_msgs::msg::Pose bot_pose_{};
  // std::string file_name = "/home/davide-work/humble_ws/wheele/status/WaypointFollower/cone1_map.yaml";
  common_.DecodeYamlToMap(file_name + "_map.yaml", map);
  common_.DecodeYamlToPose(file_name + "_pose.yaml", bot_pose_);

  // setup data
  CoreSearchTarget core_;
  OccupancyGrid oc_ = {map};
  auto bot_ = tools_.position_2_map(bot_pose_.position, oc_.map_res_, oc_.map_x0_, oc_.map_y0_);
  nav_msgs::msg::Odometry odom;
  odom.pose.pose = bot_pose_;
  BotOdom odom_ = {odom};
  // Kernel<5> kernel_{Matrix<5>{
  //     {0.0, 0.0, 0.0, 0.0, 0.0,
  //      0.0, 0.0, 0.0, 0.0, 0.0,
  //      0.11, 0.1, 0.0, 0.3, 0.25,
  //      0.45, 0.5, 1, 0.28, 0.32,
  //      0.4, 0.35, 0.7, 0.22, 0.31}}};

  // geometry_msgs::msg::Point p_target_{};
  cv::namedWindow("Image window");
  // real operation

  std::cout << "matrix\n";
  // std::cout<<oc_.matrix_.matrix_;
  std::cout << "\n";
  SearchTargetParams params_;
  params_.dev_mode_ = 5;
  params_.distance_wall_ = 10;
  params_.search_dir_ = 1; // 1 right, -1 left direction of research
  params_.angle_offset_ = 0.0;
  params_.zoom_ = 10.0;
  params_.kernel_.set_kernel(Matrix<5>{
      {0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0,
       0.11, 0.1, 0.0, 0.3, 0.25,
       0.45, 0.5, 1, 0.28, 0.32,
       0.4, 0.35, 0.7, 0.22, 0.31}});

  core_.update(bot_, oc_, odom_, params_);
  return 0;
}