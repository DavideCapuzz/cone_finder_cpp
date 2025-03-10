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

#ifndef CoreSearchTarget_H
#define CoreSearchTarget_H

class CoreSearchTarget
{
public:
	CoreSearchTarget(){};
	~CoreSearchTarget(){};
 
  geometry_msgs::msg::Point update(
    int r_bot, int c_bot, float rotation_angle, double  distance_wall, int search_dir,
    double angle_offset, Kernel<5> & kernel, int dev_mode = 0
  );
  ToolsCam tools_{};
  OccupancyGrid oc_{};
  BotOdom odom_{};
  Common common_{}; 
};

#endif
