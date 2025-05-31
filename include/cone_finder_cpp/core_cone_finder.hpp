#include <vector>
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

#include <boost/math/special_functions/round.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include "cone_finder_cpp/tools.hpp"
#include "common/common.hpp"
#include <tuple>

#ifndef CoreConeFinder_H
#define CoreConeFinder_H

struct ConeFinderParams{
  int dev_mode_{0};
  double zoom_{1.0};
};

class CoreConeFinder
{
public:
	CoreConeFinder(){};
	~CoreConeFinder(){};
 
  geometry_msgs::msg::Point update(
    cv::Mat &in_image, sensor_msgs::msg::CameraInfo & camera_info, Point &p_cam, OccupancyGrid &oc, BotOdom &odoms,
    vision_msgs::msg::BoundingBox2DArray & BB_array);
  
  void set_params(int dev_mode, double zoom);

private:
  geometry_msgs::msg::Point find_cone_pose(Point &p_cam, std::vector<double> & cone_x, OccupancyGrid &oc, BotOdom &odom);

  std::tuple<std::vector<cv::Point>, std::vector<double>, vision_msgs::msg::BoundingBox2DArray> 
  find_cone(cv::Mat &in_image, sensor_msgs::msg::CameraInfo &camera_info);

  bool pointingUp(std::vector<cv::Point> &hull_in);
  // bool find_cone(cv::Mat &in_image);
  ToolsCam tools_{};
  ConeFinderParams params_{};
  Common common_{};
};

#endif
