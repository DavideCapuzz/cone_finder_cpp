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
  ToolsCam tools_{};              // tools cone finder node 
  std::cout << "oh" << std::endl;
  nav_msgs::msg::OccupancyGrid map{};
  std::string file_name = "/home/davide-work/humble_ws/wheele/src/cone_finder_cpp/start1.yaml";
  tools_.DecodeYamlToMap(file_name,map);
  return 0;
}