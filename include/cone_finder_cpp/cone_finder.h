#include <vector>
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.

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

// next steps 
// reoreder style
// remove overlap cones
// send bounding box
// send point message


#ifndef ConeFinder_H
#define ConeFinder_H

class ConeFinder: public rclcpp::Node
{
public:
	ConeFinder();
	~ConeFinder();

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_maker_;
  rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr publisher_BB_;
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_info_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;

  void publishMarker(std::vector<cv::Point> & p_vector);
  void imageCallBack(const sensor_msgs::msg::Image::SharedPtr msg_in);
  void imageInfo_CB(const sensor_msgs::msg::CameraInfo::SharedPtr msg_in);
  void costMapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg_in);
  void get_map_indices(float x, float y, int& ix, int& iy);
  cv::Point rotatePointOnImage(const cv::Point& given_pt, const cv::Point& ref_pt, const double& angle_deg);
  cv::Point findNearestPoint(std::vector<cv::Point>& points, cv::Point P1, cv::Point P2);
  double distancePointToLine(cv::Point P, double m, double q);
  bool test_time(rclcpp::Time (&msgs_time_)[4],rclcpp::Duration max_delta);
  bool pointingUp(std::vector<cv::Point> hull_in);

  std::shared_ptr<tf2_ros::Buffer> tfBuffer;  
  
  std::vector<cv::Point> cone_coords_;
  std::vector<double> cone_x_;
  size_t count_;
  std::mutex mtx_fnc_; 
  sensor_msgs::msg::CameraInfo camera_info_;
  float map_x0_, map_y0_, map_res_;
  geometry_msgs::msg::Pose bot_pose_, goal_pose_;
  std::shared_ptr<tf2_ros::TransformListener> listener;
    
  int r_bot_{0}; // = boost::math::iround(-yg);
	int c_bot_{0}; // = boost::math::iround(xg);
  float rotation_angle_{0.0};

  rclcpp::Time msgs_time_[4];
  // 0 sub_image_
  // 1 sub_cam_info_
  // 2 sub_costmap_
  // 3 sub_odometry_

};

#endif
