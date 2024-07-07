#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "cone_finder_cpp/search_target.hpp"
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
#include <cmath>
#include <boost/math/special_functions/round.hpp>


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

SearchTarget::SearchTarget()
: Node("search_target"), count_(0)
{
    sub_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&SearchTarget::costMapCB, this, _1));
    sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&SearchTarget::odomCallback, this, _1));

    publisher_maker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/target_pos_array", 10);
    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    //timer_ = this->create_wall_timer(500ms, std::bind(&ConeFinder::timer_callback, this));
    
    tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tfBuffer->setCreateTimerInterface(timer_interface);
    listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    service_ = this->create_service<tools_nav::srv::GetTarget>(
      "/get_trg_fnd_pos", std::bind(&SearchTarget::get_search_target_server, this, _1, _2));
    
    cv::namedWindow("Image window"); 
    RCLCPP_INFO(this->get_logger(), "start search target");
}

SearchTarget::~SearchTarget(){
  cv::destroyWindow("Image window");
}


void SearchTarget::costMapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in)
{
    // https://docs.ros2.org/foxy/api/nav_msgs/msg/MapMetaData.html
    nav_msgs::msg::OccupancyGrid occupancyGrid = *msg_in;
    map_res_ = occupancyGrid.info.resolution;
    map_x0_ = occupancyGrid.info.origin.position.x;
    map_y0_ = occupancyGrid.info.origin.position.y;
    msgs_time_[2] = this->get_clock()->now();
    if (test_time(msgs_time_,rclcpp::Duration(1,0)))
    {
      //RCLCPP_INFO(this->get_logger(), " length  %d %d %f %d", occupancyGrid.info.width, occupancyGrid.info.height,occupancyGrid.info.resolution, sizeof(occupancyGrid.data[1]), sizeof(occupancyGrid.data[0]));
      count_=0;
      cv::Mat c_mat_image = cv::Mat::zeros(occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);
      cv::Mat c_mat_walls = cv::Mat::zeros(occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);
      cv::Mat c_mat_walls_gray = cv::Mat::zeros(occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);
      c_mat_image.setTo(cv::Scalar(255, 255, 255));
      c_mat_walls.setTo(cv::Scalar(255, 255, 255));
      std::vector<cv::Point> p_100;
      for (u_int i{0}; i < occupancyGrid.info.width*occupancyGrid.info.height; i++)
      {
          if (occupancyGrid.data[i]!=0)
          {
            cv::Point p = cv::Point(u_int{i%occupancyGrid.info.width},u_int{i/occupancyGrid.info.height});
            if (occupancyGrid.data[i]==100)
            {
              c_mat_image.at<cv::Vec3b>(p)=cv::Vec3b(0,0,255);
              c_mat_walls.at<cv::Vec3b>(p)=cv::Vec3b(0,0,0);
              p_100.push_back(p);
            }
            else
            {
              //c_mat_image.at<cv::Vec3b>(p)=cv::Vec3b(0,255,0);
            }
          }
      }
      // find camera position inside the costmap

      
      
      cv::Point Cam_pos = cv::Point(r_bot_,c_bot_);
      //find the closet point to the right
      cv::Point p_front = rotatePointOnImage(cv::Point(r_bot_,500), Cam_pos, rotation_angle_+90);
      //get the angle of the camera position and rotate the point for the line
      std::vector<cv::Point> p_nearest;
      std::vector<std::vector<cv::Point>> l_nearest;
      cv::cvtColor( c_mat_walls, c_mat_walls_gray, cv::COLOR_BGR2GRAY );
      if (p_100.size()>0)
      {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours( c_mat_walls_gray, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
        int i =0;
        for (std::vector<cv::Point> singel_line : contours)
        {
          cv::Point closest_point;
          double min_distance=5;
          findNearestPoint(closest_point,min_distance, singel_line, Cam_pos, p_front);
          //RCLCPP_INFO(this->get_logger(), "%d, %d %d %f ",i, closest_point.x, closest_point.y, min_distance);

          if (closest_point.x!=0 &&closest_point.y!=0 )
          {
              p_nearest.push_back(closest_point);
              l_nearest.push_back(singel_line);
          }
          i++;
        }      
      }
      //RCLCPP_INFO(this->get_logger(), "%d",l_nearest.size());
      /*
      for( size_t i = 0; i< l_nearest.size(); i++ )
      {
        for( size_t j = 0; j< l_nearest[i].size(); j++ )
        {
          RCLCPP_INFO(this->get_logger(), "%d %d %d",j, l_nearest[i][j].x ,l_nearest[i][j].y);
        }
      }
      */
      for( size_t i = 0; i< l_nearest.size(); i++ )
      {
        cv::drawContours( c_mat_image, l_nearest, (int)i, cv::Vec3b(250,0,0), 2, cv::LINE_8, 0, 0 );
        RCLCPP_INFO(this->get_logger(), "%d",l_nearest[i].size());
        //c_mat_image.at<cv::Vec3b>(l_nearest[i][0])=cv::Vec3b(0,255,0);
        int last_idx = l_nearest[i].size()-1;
        //c_mat_image.at<cv::Vec3b>(l_nearest[i][last_idx]) = cv::Vec3b(0,255,0);
        cv::Point far_point = l_nearest[i][last_idx];
        //c_mat_image.at<cv::Vec3b>(far_point) = cv::Vec3b(0,255,0);
        //c_mat_image.at<cv::Vec3b>(cv::Point(far_point.x +4,far_point.y)) = cv::Vec3b(0,255,0); 
        //RCLCPP_INFO(this->get_logger(), "%d, %d %d %f ",i, far_point.x, far_point.y, last_idx);
        p_target_=cv::Point(far_point.x +4,far_point.y);
      }

      //find the final point of the closest point 
      
      //find the perpendicular point to the closet point
      
      //cv::line (c_mat_image, Cam_pos,p_front, cv::Vec3b(255,0,255), 1);
      //cv::line (c_mat_image, Cam_pos,p_front, 4);
      //cv::imshow("Image window", c_mat_image);
      
      //cv::waitKey(3);
    }
}


cv::Point SearchTarget::rotatePointOnImage(const cv::Point& given_pt, const cv::Point& ref_pt, const double& angle_deg) {
    double    rotation_angle = angle_deg * M_PI / 180.0;
    cv::Point rotated_pt;

    rotated_pt.x = (given_pt.x - ref_pt.x) * cos(rotation_angle) - (given_pt.y - ref_pt.y) * sin(rotation_angle) + ref_pt.x;
    rotated_pt.y = (given_pt.x - ref_pt.x) * sin(rotation_angle) + (given_pt.y - ref_pt.y) * cos(rotation_angle) + ref_pt.y;

    return rotated_pt;
}

double SearchTarget::distancePointToLine(cv::Point P, double m, double q) {
    return std::abs(m * P.x - P.y + q) / std::sqrt(m * m + 1);
}

// Trova il punto nel set con la minima distanza dalla retta
void SearchTarget::findNearestPoint(cv::Point& nearestPoint, double& min_distance, std::vector<cv::Point>& points, cv::Point P1, cv::Point P2) {
    // Calcola la pendenza della retta
    double m{0.0};
    if ((P2.x - P1.x)!=0)
    {
      m = (P2.y - P1.y) / (P2.x - P1.x);
    }
    // Calcola l'intercetta y della retta
    double q = P1.y - m * P1.x;
    // Inizializza la distanza minima e il punto corrispondente
    //double minDistance = std::numeric_limits<double>::max();
    // Scansiona tutti i punti nel set e trova il punto con la minima distanza
    for (const auto& point : points) {
        double distance = distancePointToLine(point, m, q);
        if (distance < min_distance) {
            min_distance = distance;
            nearestPoint = point;
        }
    }
}

void SearchTarget::publishMarker(std::vector<cv::Point> & p_vector)
{
  //RCLCPP_INFO(this->get_logger(), " oh %f, %f" ,(p.x*map_res_+map_x0_),(p.y*map_res_+map_y0_));
  visualization_msgs::msg::MarkerArray marker_array;
  for (cv::Point p :p_vector)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "test_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = p.x * map_res_ + map_x0_;
    marker.pose.position.y = p.y * map_res_ + map_y0_;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.1;
    marker.color.a = 255.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 255.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);
  }  
  publisher_maker_->publish(marker_array); 
}

bool SearchTarget::test_time(rclcpp::Time (&msgs_time_)[4],rclcpp::Duration max_delta)
{
  //to do 
  rclcpp::Time t_ref = this->get_clock()->now();
  for (rclcpp::Time el : msgs_time_)
  {/*
    if ((t_ref - el)>max_delta){
      return false;
    }*/
    
  }
  return true;
}

void SearchTarget::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg_in)
{  
  msgs_time_[3] = this->get_clock()->now();
  bot_pose_.position = msg_in->pose.pose.position;
  bot_pose_.orientation = msg_in->pose.pose.orientation;
  // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
  geometry_msgs::msg::TransformStamped transformCamera;
  
  try{ 
    float x_init = bot_pose_.position.x;
    float y_init = bot_pose_.position.y;
    get_map_indices(x_init, y_init, c_bot_, r_bot_);
    rotation_angle_ = 2 * acos(bot_pose_.orientation.w);
    //RCLCPP_INFO(this->get_logger(), " %f ,%f ,%d, %d, %f %f",x_init, y_init, c_bot_, r_bot_, bot_pose_.orientation.w, rotation_angle_);

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Target finder: %s",ex.what());
    return;
  } 
}

void SearchTarget::get_search_target_server(const std::shared_ptr<tools_nav::srv::GetTarget::Request> request,
          std::shared_ptr<tools_nav::srv::GetTarget::Response>  response)
{
  if (p_target_.x!=0 && p_target_.y!=0)
  {
    RCLCPP_INFO(this->get_logger(), " founded !!");
    response->result=true;
    response->target.x=p_target_.x * map_res_ + map_x0_;
    response->target.y=p_target_.y * map_res_ + map_y0_;
  }
  else{
    RCLCPP_INFO(this->get_logger(), " NOT founded !!");
    response->result=false;
    response->target.x=0;
    response->target.y=0;
  }
  response->type.data="Target";
}

void SearchTarget::get_map_indices(float x, float y, int& ix, int& iy)
{
	ix = static_cast<int>(round ((x-map_x0_)/map_res_ - map_res_));
	iy = static_cast<int>(round ((y-map_y0_)/map_res_ - map_res_));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SearchTarget>());
  rclcpp::shutdown();
  return 0;
}