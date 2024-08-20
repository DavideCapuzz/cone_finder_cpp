#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "cone_finder_cpp/cone_finder.hpp"
#include "cone_finder_cpp/tools.hpp"
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
#include <cmath>
#include <boost/math/special_functions/round.hpp>


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

ConeFinder::ConeFinder()
: Node("cone_finder"), count_(0)
{
    sub_image_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/compressed", 10, std::bind(&ConeFinder::imageCallBack, this, _1));
    sub_cam_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", 10, std::bind(&ConeFinder::imageInfo_CB, this, _1));
    sub_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&ConeFinder::costMapCB, this, _1));
    sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&ConeFinder::odomCallback, this, _1));

    publisher_maker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cone_options_array", 10);
    publisher_BB_ = this->create_publisher<vision_msgs::msg::BoundingBox2DArray>("/cone/BB", 10);
    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    //timer_ = this->create_wall_timer(500ms, std::bind(&ConeFinder::timer_callback, this));
    
    tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tfBuffer->setCreateTimerInterface(timer_interface);
    listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    service_ = this->create_service<interfaces::srv::GetTarget>(
      "/get_cone_pos", std::bind(&ConeFinder::get_cone_server, this, _1, _2));
    if (dev_mode_>0)
    {
      cv::namedWindow("Image window"); 
    }
}

ConeFinder::~ConeFinder(){
  if (dev_mode_>0)
  {
    cv::destroyWindow("Image window");
  }
}


void ConeFinder::timer_callback()
{
    
}

void ConeFinder::costMapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in)
{
    // https://docs.ros2.org/foxy/api/nav_msgs/msg/MapMetaData.html
    nav_msgs::msg::OccupancyGrid occupancyGrid = *msg_in;
    map_res_ = occupancyGrid.info.resolution;
    map_x0_ = occupancyGrid.info.origin.position.x;
    map_y0_ = occupancyGrid.info.origin.position.y;
    msgs_time_[2] = this->get_clock()->now();
    if (test_time(msgs_time_,rclcpp::Duration(1,0)) && cone_x_.size()>0)
    {
      //RCLCPP_INFO(this->get_logger(), " length  %d %d %f %d", occupancyGrid.info.width, occupancyGrid.info.height,occupancyGrid.info.resolution, sizeof(occupancyGrid.data[1]), sizeof(occupancyGrid.data[0]));
      count_=0;
      cv::Mat c_mat_image = cv::Mat::zeros(occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);
      cv::Mat c_mat_walls = cv::Mat::zeros(occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);

      std::vector<cv::Point> p_100;
      tools_.grid_2_image(occupancyGrid, p_100, c_mat_image, c_mat_walls);      

      // find camera position inside the costmap
      
      cv::Point Cam_pos = cv::Point(r_bot_,c_bot_);
      cv::Point p_front = tools_.rotate_point_on_image(cv::Point(r_bot_, 500), Cam_pos, rotation_angle_);
      //get the angle of the camera position and rotate the point for the line
      std::vector<cv::Point> p_nearest;
      if (p_100.size()>0)
      {
        for (double cone : cone_x_)
        {
          cv::Point p = tools_.find_nearest_point(p_100, Cam_pos, cv::Point(p_front.x * cone + p_front.x, 500));
          p_nearest.push_back(cv::Point(p.x, p.y));
        }      
        publishMarker(p_nearest);
        p_nearest_ = p_nearest;
      }
      if (dev_mode_>0)
      {
        cv::line (c_mat_image, Cam_pos,p_front, cv::Vec3b(255,0,255), 1);
        cv::line (c_mat_image, Cam_pos,cv::Point(p_front.x*cone_x_[0]+p_front.x,500), cv::Vec3b(0,0,255), 1);
        cv::imshow("Image window", c_mat_image);
        
        cv::waitKey(3);
      }
    }
}


void ConeFinder::publishMarker(std::vector<cv::Point> & p_vector)
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
    marker.color.g = 0.0;
    marker.color.b = 255.0;
    marker_array.markers.push_back(marker);
  }  
  publisher_maker_->publish(marker_array); 
}


void ConeFinder::imageCallBack(const sensor_msgs::msg::CompressedImage::SharedPtr msg_in)
{
    msgs_time_[0] = this->get_clock()->now();
 
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg_in, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_INFO(this->get_logger(), " error !!");
      return;
    }
    cv::Mat in_image =cv_ptr->image;

    // reset timer 
    //image_time_ = now();
    // https://github.com/MicrocontrollersAndMore/Traffic_Cone_Detection_Visual_Basic/blob/master/frmMain.vb
    cv::Mat fullImageHSV;
    cv::Mat low_threshold;
    cv::Mat high_threshold;
    cv::cvtColor(in_image, fullImageHSV, CV_BGR2HSV);
    inRange(fullImageHSV, cv::Scalar(0, 130, 130), cv::Scalar(45, 255, 255), low_threshold);
    inRange(fullImageHSV, cv::Scalar(150, 135, 135), cv::Scalar(200, 255, 255), high_threshold);
    
    cv::Mat or_im;
    cv::bitwise_or (low_threshold, high_threshold, or_im);
    cv::Mat er_im;
    int erosion_size = 0;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
      cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
      cv::Point( erosion_size, erosion_size ) );

    cv::erode( or_im, er_im, element );
    cv::Mat dil_im;
    cv::erode( er_im, dil_im, element );
    cv::Mat gaus_im;
    cv::GaussianBlur( dil_im, gaus_im, cv::Size( 3, 3 ), 0, 0 );

    cv::Mat canny_im;
    cv::Canny ( gaus_im, canny_im, 160, 80); 	    

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( canny_im, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );
    
    // std::vector<std::vector<cv::Point>> poly( contours.size() );
    // std::vector<std::vector<cv::Point>> hull( contours.size() );
    std::vector<cv::Point> poly;
    std::vector<cv::Point> hull;
    std::vector<std::vector<cv::Point>> hull_refined;

    for (size_t i = 0; i < contours.size(); ++i) {
      // approx poly need to be tune  
      if (contours[i].size()>20){
      cv::approxPolyDP(contours[i], poly, 20, true);}
      else {
        poly=contours[i];
      }
      cv::convexHull( poly, hull );  
      if (hull.size() >= 3)
      {
        if (pointingUp(hull)){
          hull_refined.push_back(hull);
        }        
      }      
    }
    // remove duplicates TBD
    /*if (hull_refined.size()>1){
      remove_duplicate(hull_refined);
    }*/

    // cv::Scalar color1 = cv::Scalar( 0, 255, 0);
    // cv::Scalar color2 = cv::Scalar( 255, 0,0);
    //RCLCPP_INFO(this->get_logger(), " c %b ",hull_refined.size());
    mtx_fnc_.lock();
    cone_coords_.clear();
    cone_x_.clear();
    vision_msgs::msg::BoundingBox2DArray BB_array;
    for( size_t i = 0; i< hull_refined.size(); i++ )
    {
      //cv::drawContours( in_image, contours, (int)i, color1, 2, cv::LINE_8, hierarchy, 0 );
      //cv::drawContours( in_image, hull_refined, (int)i, color2 );
      cv::Moments m = cv::moments(hull_refined[i],true);
      cv::Point p(m.m10/m.m00, m.m01/m.m00);    
      //cv::circle(in_image, p, 10, CV_RGB(255,0,0));
      cv::Rect boundRect = cv::boundingRect( hull_refined[i] );
      //cv::rectangle( in_image, boundRect.tl(), boundRect.br(), CV_RGB(0,0,255), 2 );
      double x = (p.x - camera_info_.k[2])/camera_info_.k[0];
      //RCLCPP_INFO(this->get_logger(), " %d, %f",p.x,x);
      cone_coords_.push_back(p);
      cone_x_.push_back(x);
      vision_msgs::msg::BoundingBox2D BB;
      BB.center.position.x = p.x;
      BB.center.position.y = p.y;
      BB.size_x = boundRect.width;
      BB.size_y = boundRect.height;
      BB_array.boxes.push_back(BB);
    }
    mtx_fnc_.unlock();
    if (hull_refined.size()>0){
      publisher_BB_->publish(BB_array); 
    }
    //"/cone/BB",
    // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());*/
}

bool ConeFinder::pointingUp(std::vector<cv::Point> hull_in)
{
  // evaluate aspect ratio
  cv::Rect boundRect = cv::boundingRect( hull_in );
  double aspect_ratio = boundRect.width / boundRect.height;

  if (aspect_ratio > 0.8){
    return false;
  }
  // find the y center of the cone
  double intYCenter = boundRect.y + boundRect.height;  

  std::vector<cv::Point> listOfPointsAboveCenter;
  std::vector<cv::Point> listOfPointsBelowCenter;

  for (cv::Point p : hull_in)
  {
    if (p.y < intYCenter)
    {
      listOfPointsBelowCenter.push_back(p);
    }
    else if (p.y >= intYCenter)
    {
      listOfPointsAboveCenter.push_back(p);
    }
  }

  double intLeftMostPointBelowCenter = hull_in[0].x;
  double intRightMostPointBelowCenter = hull_in[0].x;

  for (cv::Point p : listOfPointsBelowCenter){
    if (p.x < intLeftMostPointBelowCenter){
      intLeftMostPointBelowCenter = p.x;
    }
  }

  for (cv::Point p : listOfPointsBelowCenter){
    if (p.x < intRightMostPointBelowCenter){
      intRightMostPointBelowCenter = p.x;
    }
  }

  for (cv::Point p : listOfPointsAboveCenter){
    if (p.x < intLeftMostPointBelowCenter || p.x > intRightMostPointBelowCenter)
    {
      return false;
    }
  }

  return true;      
}

bool ConeFinder::test_time(rclcpp::Time (&msgs_time_)[4],rclcpp::Duration max_delta)
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

void ConeFinder::imageInfo_CB(const sensor_msgs::msg::CameraInfo::SharedPtr msg_in)
{
  msgs_time_[1] = this->get_clock()->now();
  camera_info_ = *msg_in;
}

void ConeFinder::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg_in)
{  
  msgs_time_[3] = this->get_clock()->now();
  bot_pose_.position = msg_in->pose.pose.position;
  bot_pose_.orientation = msg_in->pose.pose.orientation;
  // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
  geometry_msgs::msg::TransformStamped transformCamera;
  
  try{
    transformCamera = tfBuffer->lookupTransform("map", "cam_frame", rclcpp::Time(0) );
    geometry_msgs::msg::Pose bot_camera{};
    geometry_msgs::msg::Pose bot_camera_tf{};    
    tf2::doTransform(bot_camera_tf, bot_camera, transformCamera);
    tools_.position_2_map(bot_pose_.position, map_res_, map_x0_, map_y0_, r_bot_, c_bot_);
    rotation_angle_ = 2 * acos(bot_pose_.orientation.w);
    //RCLCPP_INFO(this->get_logger(), " %f ,%f ,%d, %d, %f %f",x_init, y_init, c_bot_, r_bot_, bot_pose_.orientation.w, rotation_angle_);

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Cone finder: %s",ex.what());
    return;
  } 
  
}

void ConeFinder::get_cone_server(const std::shared_ptr<interfaces::srv::GetTarget::Request> request,
          std::shared_ptr<interfaces::srv::GetTarget::Response>  response)
{
  if (p_nearest_.size()>0)
  {
    RCLCPP_INFO(this->get_logger(), " founded !!");
    response->result=true;
    tools_.map_2_position(p_nearest_[0].y, p_nearest_[0].x, map_res_, map_x0_, map_y0_, response->target);
  }
  else{
    RCLCPP_INFO(this->get_logger(), " NOT founded !!");
    response->result=false;
    response->target.x=0;
    response->target.y=0;
  }
  response->type.data="Cone";
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeFinder>());
  rclcpp::shutdown();
  return 0;
}