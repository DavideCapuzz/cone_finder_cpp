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
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

SearchTarget::SearchTarget()
    : Node("search_target")
{
  // setup tf listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // parameters
  this->declare_parameter("dev_mode_", 0);
  this->declare_parameter("distance_wall_", 0.5);
  this->declare_parameter("search_dir_", 1);
  this->declare_parameter("continuos_call_back_", true);
  this->declare_parameter("angle_offset_", 0.0);

  this->declare_parameter("topic_map", "/map");
  this->declare_parameter("topic_odom", "/odom");

  std::string topic_map = this->get_parameter("topic_map").as_string();
  std::string topic_odom = this->get_parameter("topic_odom").as_string();
  params_.dev_mode_ = this->get_parameter("dev_mode_").as_int();
  params_.distance_wall_ = this->get_parameter("distance_wall_").as_double();
  params_.search_dir_ = this->get_parameter("search_dir_").as_int();
  continuos_call_back_ = this->get_parameter("continuos_call_back_").as_bool();
  RCLCPP_INFO(this->get_logger(), "dev_mode  %d", params_.dev_mode_);

  // ros interfaaces
  sub_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic_map, 10, std::bind(&SearchTarget::costmap_cb, this, _1));
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_odom, 10, std::bind(&SearchTarget::odom_cb, this, _1));

  srv_get_target_ = this->create_service<interfaces::srv::GetTarget>(
      "/get_trg_fnd_pos", std::bind(&SearchTarget::get_search_target_server, this, _1, _2));
  srv_save_status_ = this->create_service<interfaces::srv::SaveStatus>(
      "/save_status_search_target", std::bind(&SearchTarget::save_status_server, this, _1, _2));

  publisher_maker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/target_pos_array", 10);

  main_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(25),
      std::bind(&SearchTarget::continuosCallback, this));

  // parameters
  params_.kernel_.set_kernel(Matrix<5>{
      {0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0,
       0.11, 0.1, 0.0, 0.3, 0.25,
       0.45, 0.5, 1, 0.28, 0.32,
       0.4, 0.35, 0.7, 0.22, 0.31}});

  // intialize all
  if (params_.dev_mode_ > 0)
  {
    cv::namedWindow("Image window");
  }

  msgs_time_.resize(2);

  RCLCPP_INFO(this->get_logger(), "start search target");
}

SearchTarget::~SearchTarget()
{
  if (params_.dev_mode_ > 0)
  {
    cv::destroyWindow("Image window");
  }
}

void SearchTarget::continuosCallback()
{
  // check if the messages is update
  if (continuos_call_back_ && common_.check_time(msgs_time_)) // TODO: check if all the information are updated
  {
    p_target_ = core_.update(
        r_bot_, c_bot_, core_.odom_.rotation_angle_);
    if (p_target_.x == 0 && p_target_.y == 0)
    {
      p_target_.x = p_target_.x + 1.5 * cos((core_.odom_.rotation_angle_ - 90) * M_PI / 180.0);
      p_target_.y = p_target_.y + 1.5 * sin((core_.odom_.rotation_angle_ - 90) * M_PI / 180.0);
    }
    std::vector<geometry_msgs::msg::Point> pp;
    pp.push_back(p_target_);
    publish_marker(pp);
  }
}

void SearchTarget::costmap_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in)
{
  if (msg_in->info.height != 0 && msg_in->info.width != 0)
  {
    core_.oc_ = {*msg_in};
  }
  msgs_time_[0] = std::chrono::system_clock::now();
}

void SearchTarget::publish_marker(std::vector<geometry_msgs::msg::Point> &p_vector)
{
  std_msgs::msg::Header h;
  h.stamp = now();
  std::string name{"test_marker"};
  publisher_maker_->publish(common_.create_marker_array(
      p_vector, 0.0, 255.0, 0.0, name, h));
}

void SearchTarget::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg_in)
{
  msgs_time_[1] = std::chrono::system_clock::now();

  // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
  geometry_msgs::msg::TransformStamped transformCamera;
  core_.odom_ = {*msg_in};

  try
  {
    std::tie(r_bot_, c_bot_) = tools_.position_2_map(core_.odom_.pose_.position, core_.oc_.map_res_, core_.oc_.map_x0_, core_.oc_.map_y0_);
    // RCLCPP_INFO(this->get_logger(), "rot %f or %f", rotation_angle_, bot_pose_.orientation.w);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(get_logger(), "Target finder: %s", ex.what());
    return;
  }
}

void SearchTarget::get_search_target_server(const std::shared_ptr<interfaces::srv::GetTarget::Request> request,
                                            std::shared_ptr<interfaces::srv::GetTarget::Response> response)
{
  (void)request;
  if (p_target_.x != 0 && p_target_.y != 0)
  {
    // RCLCPP_INFO(this->get_logger(), " founded !!");
    response->result = true;
    response->target = p_target_;
    if (params_.dev_mode_ > 1)
    {
      RCLCPP_INFO(this->get_logger(), "%f %f", p_target_.x * core_.oc_.map_res_ + core_.oc_.map_x0_, p_target_.y * core_.oc_.map_res_ + core_.oc_.map_y0_);
    }
  }
  else
  {
    // RCLCPP_INFO(this->get_logger(), " NOT founded !!");
    response->result = false;
    response->target.x = 0;
    response->target.y = 0;
  }
  response->type.data = "Target";
}

void SearchTarget::save_status_server(const std::shared_ptr<interfaces::srv::SaveStatus::Request> request,
                                      std::shared_ptr<interfaces::srv::SaveStatus::Response> response)
{
  if (common_.check_time(msgs_time_))
  {
    common_.WriteMapToYaml(request->name, core_.oc_.grid_);
    common_.WritePoseToYaml(request->name, core_.odom_.pose_);
    // RCLCPP_INFO(this->get_logger(), " founded !!");
    response->result = true;
  }
  else
  {
    response->result = false;
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SearchTarget>());
  rclcpp::shutdown();
  return 0;
}