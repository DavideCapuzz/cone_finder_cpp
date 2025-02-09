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
: Node("cone_finder")
{
  this->declare_parameter("dev_mode_", 0);
  this->declare_parameter("continuos_call_back_", true);
  this->declare_parameter("topic_image", "/camera/compressed");
  this->declare_parameter("topic_camera_info", "/camera/camera_info");
  this->declare_parameter("topic_map", "/map");
  this->declare_parameter("topic_odom", "/odom");

  dev_mode_ = this->get_parameter("dev_mode_").as_int();
  continuos_call_back_ = this->get_parameter("continuos_call_back_").as_bool();
  std::string topic_image = this->get_parameter("topic_image").as_string();
  std::string topic_camera_info = this->get_parameter("topic_camera_info").as_string();
  std::string topic_map = this->get_parameter("topic_map").as_string();
  std::string topic_odom = this->get_parameter("topic_odom").as_string();

  sub_image_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    topic_image, 10, std::bind(&ConeFinder::imageCallBack, this, _1));
  sub_cam_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    topic_camera_info, 10, std::bind(&ConeFinder::imageInfo_CB, this, _1));
  sub_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_map, 10, std::bind(&ConeFinder::costMapCB, this, _1));
  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
    topic_odom, 10, std::bind(&ConeFinder::odomCallback, this, _1));

  publisher_maker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cone_options_array", 10);
  publisher_BB_ = this->create_publisher<vision_msgs::msg::BoundingBox2DArray>("/cone/BB", 10);
  
  tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());

  main_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(25),
    std::bind(&ConeFinder::continuosCallback, this)); 

  tfBuffer->setCreateTimerInterface(timer_interface);
  listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  service_ = this->create_service<interfaces::srv::GetTarget>(
    "/get_cone_pos", std::bind(&ConeFinder::get_cone_server, this, _1, _2));


  if (dev_mode_>0)
  {
    cv::namedWindow("Image window"); 
  }
  msgs_time_.resize(3);
}

ConeFinder::~ConeFinder(){
  if (dev_mode_>0)
  {
    cv::destroyWindow("Image window");
  }
}


void ConeFinder::continuosCallback()
{ 

  vision_msgs::msg::BoundingBox2DArray BB_array;
  if (continuos_call_back_ && common_.check_time(msgs_time_))
  {
    goal_pose_ = core_.update(in_image_, camera_info_, r_bot_, c_bot_, BB_array, dev_mode_);
  }
  if (BB_array.boxes.size()>0){
      publisher_BB_->publish(BB_array); 
    }    
}

void ConeFinder::costMapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in)
{
    // https://docs.ros2.org/foxy/api/nav_msgs/msg/MapMetaData.html
    if (msg_in->info.height !=0 && msg_in->info.width !=0)
    {
      core_.oc_ = {*msg_in};
      msgs_time_[2] = std::chrono::system_clock::now();
    }    
}


void ConeFinder::publishMarker(std::vector<cv::Point> & p_vector)
{
  //RCLCPP_INFO(this->get_logger(), " oh %f, %f" ,(p.x*oc_.map_res_+oc_.oc_.map_x0_),(p.y*oc_.map_res_+oc_.map_y0_));
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
    marker.pose.position.x = p.x * core_.oc_.map_res_ + core_.oc_.map_x0_;
    marker.pose.position.y = p.y * core_.oc_.map_res_ + core_.oc_.map_y0_;
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
    msgs_time_[0] = std::chrono::system_clock::now();
 
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
    in_image_ =cv_ptr->image;

}

void ConeFinder::imageInfo_CB(const sensor_msgs::msg::CameraInfo::SharedPtr msg_in)
{
  msgs_time_[1] = std::chrono::system_clock::now();
  camera_info_ = *msg_in;
}

void ConeFinder::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg_in)
{  
  msgs_time_[3] = std::chrono::system_clock::now();
  // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
  core_.odom_ = {*msg_in};
  try{  
    geometry_msgs::msg::TransformStamped transformCamera;
    transformCamera = tfBuffer->lookupTransform("map", "cam_frame", rclcpp::Time(0) );
    geometry_msgs::msg::Pose bot_camera{};
    geometry_msgs::msg::Pose bot_camera_tf{};
    tf2::doTransform(bot_camera, bot_camera_tf, transformCamera); // get the pose of the camera
    // RCLCPP_INFO(this->get_logger(), "cam_tf %f %f %f", bot_camera_tf.position.x, bot_camera_tf.position.y, bot_camera_tf.position.z );
    // RCLCPP_INFO(this->get_logger(), "cam    %f %f %f", bot_camera.position.x, bot_camera.position.y, bot_camera.position.z );
    tools_.position_2_map(bot_camera_tf.position, core_.oc_.map_res_, core_.oc_.map_x0_, core_.oc_.map_y0_, r_bot_, c_bot_);
    
    //RCLCPP_INFO(this->get_logger(), " %f ,%f ,%d, %d, %f %f",x_init, y_init, c_bot_, r_bot_, bot_pose_.orientation.w, rotation_angle_);

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Cone finder: %s",ex.what());
    return;
  } 
  
}

void ConeFinder::get_cone_server(const std::shared_ptr<interfaces::srv::GetTarget::Request> request,
          std::shared_ptr<interfaces::srv::GetTarget::Response>  response)
{
  if (goal_pose_.x>0)
  {
    RCLCPP_INFO(this->get_logger(), " founded !!");
    response->result=true;
    response->target.x= goal_pose_.x;
    response->target.y=goal_pose_.y;
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