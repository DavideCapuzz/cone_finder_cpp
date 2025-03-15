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
#include <tuple>

#ifndef CoreSearchTarget_H
#define CoreSearchTarget_H

struct SearchTargetParams{
  double  distance_wall_{};
  int search_dir_{};
  double angle_offset_{};
  Kernel<5> kernel_{};
  int dev_mode_{0};
  double zoom_{1.0};
};

class CoreSearchTarget
{
public:
	CoreSearchTarget(){};
	~CoreSearchTarget(){};
 
  std::tuple<bool, geometry_msgs::msg::Point> update(
    Point &bot_rc, OccupancyGrid &oc, BotOdom &odom, SearchTargetParams& params
  );
private:
  ToolsCam tools_{};
  Common common_{};
};

#endif
