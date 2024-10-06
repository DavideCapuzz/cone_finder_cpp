#include <array>
#include <cmath>
#include <iostream>
#include <vector>
#include <iostream>
#include <fstream>

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
#include "interfaces/srv/get_target.hpp"
#include "interfaces/srv/save_cost_map.hpp"

#include "cone_finder_cpp/tools.hpp"
#include "common/common.hpp"

#ifndef SearchTarget_H
#define SearchTarget_H

class SearchTarget: public rclcpp::Node
{
public:
	SearchTarget();
	~SearchTarget();

private:
  // ros interface
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;

  rclcpp::Service<interfaces::srv::GetTarget>::SharedPtr srv_get_target_;
  rclcpp::Service<interfaces::srv::SaveCostMap>::SharedPtr srv_save_cost_map_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_maker_;
  rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr publisher_BB_;

  //! Ros2 Timer variable.
  rclcpp::TimerBase::SharedPtr main_timer_;

  // input callback
  void costmap_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in);
  void costmap_cb_old(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in);
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg_in); 

  void get_search_target_server(const std::shared_ptr<interfaces::srv::GetTarget::Request> request,
          std::shared_ptr<interfaces::srv::GetTarget::Response> response);
  void save_cost_map_server(const std::shared_ptr<interfaces::srv::SaveCostMap::Request> request,
          std::shared_ptr<interfaces::srv::SaveCostMap::Response> response);

  // internal functions
  void publish_marker(std::vector<geometry_msgs::msg::Point> & p_vector);
  bool check_time(std::array<std::chrono::time_point<std::chrono::system_clock>, 2> & msgs_time);

  void continuosCallback();

  // input
  OccupancyGrid oc_{};
  geometry_msgs::msg::Pose bot_pose_{}, goal_pose_{};
  
  
  int r_bot_{0};                  // row the base link of the robot
  int c_bot_{0};                  // column the base link of the robot
  float rotation_angle_{0.0};     // heading of the robot

  // internal data
  ToolsCam tools_{};              // tools cone finder node 
  Common common_{};              // tools cone finder node 
  std::array<std::chrono::time_point<std::chrono::system_clock>, 2> msgs_time_;
  // output
  geometry_msgs::msg::Point p_target_{};          // next target point

  // parameters 
  int dev_mode_{0};                 // dev level 0 - 3
  double distance_wall_{10};        // parameter of the algorithm
  int search_dir_{1};               // 1 right, -1 left direction of research
  bool continuos_call_back_{true};  // if true the continuos callback will work
  double angle_offset_{0.0};
  

//   Kernel<5> kernel_{Matrix<5>{
//     {0.0, 0.0, 0.25, 0.22, 0.31, 
//     0.0,  0.0, 0.3, 0.28, 0.32, 
//     0.0,  0.0, 0.0, 1, 0.7,
//     0.0,  0.0, 0.55, 0.45, 0.5,
//     0.0, 0.0,  0.65, 0.35, 0.4}}};

  Kernel<5> kernel_{Matrix<5>{
    {0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0,  0.0, 0.0, 0.0, 0.0, 
    0.0,  0.0, 0.0, 0.3, 0.25,
    0.45,  0.5, 1, 0.28, 0.32,
    0.4, 0.35,  0.7, 0.22, 0.31}}};

rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_visualize_image_;
};

#endif

