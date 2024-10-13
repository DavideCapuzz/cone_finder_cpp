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

  // ros interfaaces
  sub_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&SearchTarget::costmap_cb, this, _1));
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&SearchTarget::odom_cb, this, _1));

  srv_get_target_ = this->create_service<interfaces::srv::GetTarget>(
      "/get_trg_fnd_pos", std::bind(&SearchTarget::get_search_target_server, this, _1, _2));
  srv_save_cost_map_ = this->create_service<interfaces::srv::SaveCostMap>(
      "/save_cost_map", std::bind(&SearchTarget::save_cost_map_server, this, _1, _2));

  publisher_maker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/target_pos_array", 10);

  main_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(25),
    std::bind(&SearchTarget::continuosCallback, this));

  // parameters
  this->declare_parameter("dev_mode_", 0);
  this->declare_parameter("distance_wall_", 0.5);
  this->declare_parameter("search_dir_", 1);
  this->declare_parameter("continuos_call_back_", true);
  this->declare_parameter("angle_offset_", 0.0);

  dev_mode_ = this->get_parameter("dev_mode_").as_int();
  distance_wall_ = this->get_parameter("distance_wall_").as_double();
  search_dir_ = this->get_parameter("search_dir_").as_int();
  continuos_call_back_ = this->get_parameter("continuos_call_back_").as_bool();
  angle_offset_ = this->get_parameter("angle_offset_").as_double();
  RCLCPP_INFO(this->get_logger(), "dev_mode  %d", dev_mode_);

  // intialize all
  if (dev_mode_ > 0)
  {
    cv::namedWindow("Image window");
  }

  RCLCPP_INFO(this->get_logger(), "start search target");
}

SearchTarget::~SearchTarget()
{
  if (dev_mode_ > 0)
  {
    cv::destroyWindow("Image window");
  }
}

void SearchTarget::continuosCallback()
{
  // check if the messages is update  
  if (continuos_call_back_ && check_time(msgs_time_)) // TODO: check if all the information are updated
  {
    p_target_ = core_.update(
      r_bot_, c_bot_, rotation_angle_, distance_wall_, search_dir_,
      angle_offset_, kernel_, dev_mode_);

    std::vector<geometry_msgs::msg::Point> pp;
    pp.push_back(p_target_);
    publish_marker(pp);
  }
}

bool SearchTarget::check_time(std::array<std::chrono::time_point<std::chrono::system_clock>, 2> & msgs_time)
{
  bool flag{true};
  for (size_t i = 0; i < msgs_time.size(); ++i) {
        // Capture the current time and store it in the array+        
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now()-msgs_time[i]).count()>300)
        {
          flag = false;
        }
    }
  return flag;
}

void SearchTarget::costmap_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in)
{
  core_.oc_ = {*msg_in};  
  msgs_time_[0] = std::chrono::system_clock::now();;
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
  msgs_time_[1] = std::chrono::system_clock::now();;
  bot_pose_.position = msg_in->pose.pose.position;
  bot_pose_.orientation = msg_in->pose.pose.orientation;
  // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
  geometry_msgs::msg::TransformStamped transformCamera;

  try
  {
    tools_.position_2_map(bot_pose_.position, core_.oc_.map_res_, core_.oc_.map_x0_, core_.oc_.map_y0_, r_bot_, c_bot_);
    rotation_angle_ = 2 * acos(bot_pose_.orientation.w) * 180 / 3.14;
    //RCLCPP_INFO(this->get_logger(), "rot %f or %f", rotation_angle_, bot_pose_.orientation.w);
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
    if (dev_mode_ > 0)
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

void SearchTarget::save_cost_map_server(const std::shared_ptr<interfaces::srv::SaveCostMap::Request> request,
                                            std::shared_ptr<interfaces::srv::SaveCostMap::Response> response)
{  
  if (check_time(msgs_time_))
  {
    tools_.WriteMapToYaml(request->name,bot_pose_ , core_.oc_.grid_);
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