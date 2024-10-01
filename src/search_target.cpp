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

  dev_mode_ = this->get_parameter("dev_mode_").as_int();
  distance_wall_ = this->get_parameter("distance_wall_").as_double();
  search_dir_ = this->get_parameter("search_dir_").as_int();
  continuos_call_back_ = this->get_parameter("continuos_call_back_").as_bool();
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
  if (continuos_call_back_) // TODO: check if all the information are updated
  {
    // find camera position inside the costmap
    cv::Point bot_pos = cv::Point(r_bot_, c_bot_);
    // find the closet point, 1 right, -1 left
    cv::Point p_lateral = tools_.rotate_point_on_image(cv::Point(r_bot_, search_dir_ * 500), bot_pos, rotation_angle_);
    // se a front point of the direction of the robot
    cv::Point p_front = tools_.rotate_point_on_image(cv::Point(r_bot_, c_bot_ + 5), bot_pos, rotation_angle_ - 90);
    if (dev_mode_ > 0)
    {
      RCLCPP_INFO(this->get_logger(), "%d, %d %d %d ", bot_pos.x, bot_pos.y, p_lateral.x, p_lateral.y);
    }
    cv::Point closest_point;
    if (oc_.p_100_.size() > 0) // check if the cost map has some obstacles
    {
      double distance{0};
      // groups all the lines in contours
      cv::Point closest_point;
      tools_.find_nearest_point_2_segment2(closest_point, distance, oc_.p_100_, bot_pos, p_lateral);
      if (dev_mode_ > 0)
      {
        RCLCPP_INFO(this->get_logger(), "cp %d %d %f ", closest_point.x, closest_point.y, tools_.distance_points(bot_pos, closest_point) );
      }

      std::vector<cv::Point> line;

      if (tools_.distance_points(bot_pos, closest_point) > 0)
      {
        if (dev_mode_ > 0)
        {
          oc_.im_occgrid_.at<cv::Vec3b>(closest_point) = cv::Vec3b(250, 0, 0);
        }
        bool result{false};
        double next_i = closest_point.x;
        double next_j = closest_point.y;
        //kernel_.flip();     
        kernel_.set_angle(rotation_angle_);
        double r {};
        do
        {
          if (line.size()>3)
          {            
            if ((line[line.size()-3].x - line[line.size()-2].x)!=0)
            {
              r = atan((line[line.size()-3].y - line[line.size()-2].y)/(line[line.size()-3].x - line[line.size()-2].x)) * 180 / 3.14;
            }
            else if ((line[line.size()-3].y - line[line.size()-2].y)>0)
            {
              r = 90;
            } else {
              r = 270;
            }            
            kernel_.set_angle(r);
          }
          Matrix<5> c_m{
              {0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0}};
          for (int i = -2; i < 2; i++)
          {
            for (int j = -2; j < 2; j++)
            {
              c_m[2 + i][2 + j] = oc_.matrix_.matrix_(static_cast<int>(next_i) + i, static_cast<int>(next_j) + j);
              oc_.im_occgrid_.at<cv::Vec3b>(
                cv::Point(static_cast<int>(next_i) +i, static_cast<int>(next_j) + j)) = cv::Vec3b(250*c_m[2 + i][2 + j], 0, 0);
            }
          }
          double i_r;
          double j_r;
          std::tie(result, i_r, j_r) = kernel_.evalute_next_point(c_m);
          
          next_i = next_i + i_r-2;
          next_j = next_j + j_r-2;

          if (result){
            line.push_back(cv::Point(next_i, next_j));
          }
          if (dev_mode_ > 0)
          {
            RCLCPP_INFO(this->get_logger(), "r %d %f %f", result, next_i, next_j);
          }

          if (dev_mode_ > 0)
          {
            oc_.im_occgrid_.at<cv::Vec3b>(cv::Point(next_i, next_j)) = cv::Vec3b(250, 0, 0);
          }
        } while (result);
        RCLCPP_INFO(this->get_logger(), "rot %f %f %f", rotation_angle_, next_i, next_j);
        if (line.size()>5)
        {
          int last_idx = line.size() - 1;  
          cv::Point p_t1 = line.back();;
          cv::Point p_t2 = (line[last_idx-1]+line[last_idx-2]+line[last_idx-3])/3;
            

          // find the point that is at a distance x to the fartest point of the robot in the direction of the research
          cv::Point p_t3 = tools_.find_points_at_distance_X2(p_t1, p_t2, distance_wall_, bot_pos); // not working well need to fix
          // translate the point from image to occupancy grid
          //cv::Point p_t = tools_.cvpoint_2_grid(p_t3, oc_.grid_.info.height);
          tools_.map_2_position(p_t3.x, p_t3.y, oc_.map_res_, oc_.map_x0_, oc_.map_y0_, p_target_);
          if (dev_mode_ > 0)
          {
            cv::line (oc_.im_occgrid_, p_t1, p_t3, cv::Vec3b(255, 0, 255), 1); // purple
            cv::circle (oc_.im_occgrid_, p_t1, 4, cv::Vec3b(0, 255, 255), 3, cv::LINE_8); // yellow
            cv::circle (oc_.im_occgrid_, p_t2, 4, cv::Vec3b(0, 255, 0), 3, cv::LINE_8); // green
            cv::circle (oc_.im_occgrid_, p_t3, 4, cv::Vec3b(255, 255, 0), 3, cv::LINE_8); // liht blue
          }
        }
      }
    }

    std::vector<geometry_msgs::msg::Point> pp;
    pp.push_back(p_target_);
    publish_marker(pp);

    if (dev_mode_ > 0)
    {
      cv::line(oc_.im_occgrid_, bot_pos, p_lateral, cv::Vec3b(255, 0, 0), 1); // blue
      cv::line(oc_.im_occgrid_, bot_pos, p_front, cv::Vec3b(0, 0, 255), 1);   // red

      cv::imshow("Image window", oc_.im_occgrid_);

      cv::waitKey(3);      
    }
  }
}

void SearchTarget::costmap_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in)
{
  oc_ ={*msg_in};  
  // msgs_time_[2] = std::chrono::system_clock;
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
  msgs_time_[3] = this->get_clock()->now();
  bot_pose_.position = msg_in->pose.pose.position;
  bot_pose_.orientation = msg_in->pose.pose.orientation;
  // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
  geometry_msgs::msg::TransformStamped transformCamera;

  try
  {
    tools_.position_2_map(bot_pose_.position, oc_.map_res_, oc_.map_x0_, oc_.map_y0_, r_bot_, c_bot_);
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
      RCLCPP_INFO(this->get_logger(), "%f %f", p_target_.x * oc_.map_res_ + oc_.map_x0_, p_target_.y * oc_.map_res_ + oc_.map_y0_);
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
  if (tools_.tryWriteMapToFile(request->name, oc_.grid_))
  {
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