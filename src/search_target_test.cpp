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


int main()
{
  // extarct data from file
  ToolsCam tools_{};              // tools cone finder node 
  std::cout << "oh" << std::endl;
  nav_msgs::msg::OccupancyGrid map{};
  geometry_msgs::msg::Pose bot_pose_{};
  std::string file_name = "/home/davide-work/humble_ws/wheele/src/cone_finder_cpp/start5.yaml";
  tools_.DecodeYamlToMap(file_name,bot_pose_,map);

  // setup data
  CoreSearchTarget core_;
  int r_bot_{0};                  // row the base link of the robot
  int c_bot_{0};                  // column the base link of the robot
  float rotation_angle_{0.0};     // heading of the robot
  core_.oc_ = {map};  
  tools_.position_2_map(bot_pose_.position, core_.oc_.map_res_, core_.oc_.map_x0_, core_.oc_.map_y0_, r_bot_, c_bot_);
  rotation_angle_ = 2 * acos(bot_pose_.orientation.w) * 180 / 3.14;

  Kernel<5> kernel_{Matrix<5>{
    {0,	0, 0,	0,	0,	
0,	0,	0,	0,	0,	
0.25,	0.3,	0,	0.1,	0.11,	
0.32,	0.28,	1,	0.5,	0.45,	
0.31,	0.22,	0.7,	0.35,	0.4}}};

  int dev_mode_ = 5;
  double distance_wall_{10};        // parameter of the algorithm
  int search_dir_{1};               // 1 right, -1 left direction of research
  double angle_offset_{0.0};

  geometry_msgs::msg::Point p_target_{};  
  cv::namedWindow("Image window");
  // real operation

  std::cout<<"matrix\n";
  // std::cout<<core_.oc_.matrix_.matrix_;
  std::cout<<"\n";

  // kernel_.flip();

  p_target_ = core_.update(
      r_bot_, c_bot_, rotation_angle_, distance_wall_, search_dir_,
      angle_offset_, kernel_, dev_mode_);
  return 0;
}