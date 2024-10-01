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

#ifndef ToolsCam_H
#define ToolsCam_H

class ToolsCam
{
public:
	ToolsCam();
	~ToolsCam();
 
  void position_2_map(
    geometry_msgs::msg::Point p, float map_res, float map_x0, float map_y0, int& ix, int& iy);

  void map_2_position(
    int ix, int iy, float map_res, float map_x0,  float map_y0, geometry_msgs::msg::Point& p);

  cv::Point rotate_point_on_image(
    const cv::Point& given_pt, const cv::Point& ref_pt, const double& angle_deg);
  cv::Point find_nearest_point(
    std::vector<cv::Point>& points, cv::Point P1, cv::Point P2);

  void find_nearest_point_2_line(
    cv::Point& nearestPoint, double& min_distance, std::vector<cv::Point>& points, cv::Point P1, cv::Point P2);
  void find_nearest_point_2_segment(
    cv::Point& nearestPoint, double& min_distance, std::vector<cv::Point>& points, cv::Point P1, cv::Point P2);
  void find_nearest_point_2_segment2(
    cv::Point& nearestPoint, double& min_distance, std::vector<cv::Point>& points, cv::Point P1, cv::Point P2);
  void closest_point_on_segment(
    cv::Point& nearestPoint, double& min_distance, cv::Point P0, cv::Point P1, cv::Point P2);
  double distance_points(
    cv::Point P1, cv::Point P0);
  cv::Point find_points_at_distance_X(
    const cv::Point& A, const cv::Point& B, double x, const cv::Point& Pbot);
  cv::Point find_points_at_distance_X2(
    const cv::Point& A, const cv::Point& B, double d, const cv::Point& Pbot);

  void grid_2_image(
    nav_msgs::msg::OccupancyGrid &occupancyGrid, std::vector<cv::Point> &p_100, cv::Mat &c_mat_image, cv::Mat &c_mat_walls);

  cv::Point grid_2_cvpoint(int grid_index, size_t grid_width, size_t grid_height);
  double distance_point_2_line(
    cv::Point P, double m, double q);

  bool test_time(
    rclcpp::Time (&msgs_time_)[4],rclcpp::Duration max_delta);  

  cv::Point cvpoint_2_grid(cv::Point p, size_t grid_height);

  bool tryWriteMapToFile(std::string & name,nav_msgs::msg::OccupancyGrid & map);
};

#endif
