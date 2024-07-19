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
    
    this->declare_parameter("dev_mode_", false);
    this->declare_parameter("distance_wall_", 0.5);

    dev_mode_ = this->get_parameter("dev_mode_").as_bool();
    distance_wall_ = this->get_parameter("distance_wall_").as_double();
    RCLCPP_INFO(this->get_logger(), "dev_mode  %d",dev_mode_ );

    if (dev_mode_)
    {
      cv::namedWindow("Image window"); 
    }    
    RCLCPP_INFO(this->get_logger(), "start search target");


}

SearchTarget::~SearchTarget(){
  if (dev_mode_)
  {
    cv::destroyWindow("Image window");
  }
}


void SearchTarget::costMapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in)
{
    nav_msgs::msg::OccupancyGrid occupancyGrid = *msg_in;
    // get map resolution and and position
    map_res_ = occupancyGrid.info.resolution;
    map_x0_ = occupancyGrid.info.origin.position.x;
    map_y0_ = occupancyGrid.info.origin.position.y;

    // check if the messages is update
    msgs_time_[2] = this->get_clock()->now();
    if (test_time(msgs_time_,rclcpp::Duration(1, 0)))
    {
      cv::Mat c_mat_image = cv::Mat::zeros(occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);
      cv::Mat c_mat_walls = cv::Mat::zeros(occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);

      // create image occupancy grid gray scale to fine countours
      cv::Mat c_mat_walls_gray = cv::Mat::zeros(occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);
      c_mat_image.setTo(cv::Scalar(255, 255, 255));
      c_mat_walls.setTo(cv::Scalar(255, 255, 255));

      std::vector<cv::Point> p_100;

      // extract from the message cost map the xy of the points
      for (u_int i{0}; i < occupancyGrid.info.width*occupancyGrid.info.height; i++)
      {
          if (occupancyGrid.data[i]!=0)
          {
            cv::Point p = cv::Point(u_int{i % occupancyGrid.info.width}, occupancyGrid.info.height - u_int{i / occupancyGrid.info.height});
            if (occupancyGrid.data[i] == 100)
            {
              if (dev_mode_)
              {
                c_mat_image.at<cv::Vec3b>(p) = cv::Vec3b(0, 0, 255);
                c_mat_walls.at<cv::Vec3b>(p) = cv::Vec3b(0, 0, 0);
              }              
              p_100.push_back(p);
            }
            else
            {
              // c_mat_image.at<cv::Vec3b>(p)=cv::Vec3b(0,255,0);
            }
          }
      }
      
      // find camera position inside the costmap     
      cv::Point bot_pos = cv::Point(r_bot_, c_bot_);
      // find the closet point to the right
      cv::Point p_lateral = rotatePointOnImage(cv::Point(r_bot_, 500), bot_pos, rotation_angle_ - 90);
      cv::Point p_front = rotatePointOnImage(cv::Point(r_bot_, 5), bot_pos, rotation_angle_);
      RCLCPP_INFO(this->get_logger(), "%d, %d %d %f ", bot_pos.x, bot_pos.y, p_lateral.x, p_lateral.y);
      // get the angle of the camera position and rotate the point for the line
      std::vector<cv::Point> closest_line;
      cv::Point closest_point;
      cv::cvtColor(c_mat_walls, c_mat_walls_gray, cv::COLOR_BGR2GRAY);
      if (p_100.size() > 0)
      {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(c_mat_walls_gray, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
        int i{0};
        int idx_cont{0};
        double min_distance_line{std::numeric_limits<double>::max()};
        double min_distance_robot{std::numeric_limits<double>::max()};
        for (std::vector<cv::Point> singel_line : contours)
        {
          cv::Point cp;
          double distance;
          findNearestPointToSegment2(cp, distance, singel_line, bot_pos, p_lateral);
          RCLCPP_INFO(this->get_logger(), "%d, %d %d %f ", i, cp.x, cp.y, distance);
          double d_robot = distancePoints(cp, bot_pos);
          if ((min_distance_line > distance) && (min_distance_robot > d_robot) && (cp.x != 0 && cp.y != 0) && (singel_line.size() !=0))
          {
            closest_line = singel_line;
            min_distance_line = distance;
            idx_cont = i;
          }
          i++;
        }
        
        if (dev_mode_)
        {
          RCLCPP_INFO(this->get_logger(), "indx line  %d", idx_cont);
          RCLCPP_INFO(this->get_logger(), "%ld", closest_line.size());
          cv::drawContours( c_mat_image, contours, idx_cont, cv::Vec3b(250,0,0), 2, cv::LINE_8, 0, 0);
        }        
        
        int last_idx = closest_line.size()-1;
        cv::Point far_point = closest_line[last_idx];
        cv::Point p_t1{};
        cv::Point p_t2{};
        if (distancePoints(p_front, far_point) > distancePoints(bot_pos, far_point))
        {
          p_t1 = closest_line[0];
          p_t2 = closest_line[2];
        } else {
          p_t1 = closest_line[last_idx];
          p_t2 = closest_line[last_idx-2];
        }
        
        
        if (dev_mode_)
        {
          c_mat_image.at<cv::Vec3b>(cv::Point(p_t1.x + 4,occupancyGrid.info.height - p_t1.y)) = cv::Vec3b(0,255,0); 
        }
        //RCLCPP_INFO(this->get_logger(), "%d, %d %d %f ",i, far_point.x, far_point.y, last_idx);

        // non funziona 
        cv::Point p_t3 = findPointsAtDistanceX2(p_t1, p_t2, distance_wall_, bot_pos);
        p_target_=cv::Point(p_t3.x,occupancyGrid.info.height - p_t3.y);    
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
      std::vector<cv::Point> pp;
      pp.push_back(p_target_);
      publishMarker(pp);

      //find the final point of the closest point 
      
      //find the perpendicular point to the closet point
      if (dev_mode_)
      {
        cv::line (c_mat_image, bot_pos, p_lateral, cv::Vec3b(255,0,255), 1);
        cv::line (c_mat_image, bot_pos, p_lateral, 4);
        cv::imshow("Image window", c_mat_image);
      
        cv::waitKey(3);
      }
      
    }
}

cv::Point SearchTarget::findPointsAtDistanceX(const cv::Point& A, const cv::Point& B, double x, const cv::Point& Pbot) {
    // Calculate the distance between A and B
    double d = distancePoints(A, B);

    // Check if the distance is possible
    if (d > 2 * x) {
        throw std::runtime_error("No such point exists.");
    }

    // Calculate the midpoint between A and B
    double mid_x = (A.x + B.x) / 2;
    double mid_y = (A.y + B.y) / 2;

    // Calculate the half distance along the line joining A and B
    double h = std::sqrt(x * x - (d / 2) * (d / 2));

    // Calculate the unit vector along the line joining A and B
    double ux = (B.x - A.x) / d;
    double uy = (B.y - A.y) / d;

    // Calculate the unit vector perpendicular to the line joining A and B
    double vx = -uy;
    double vy = ux;

    // Calculate the intersection points
    cv::Point P1 = { mid_x + h * vx, mid_y + h * vy };
    cv::Point P2 = { mid_x - h * vx, mid_y - h * vy };

    if (distancePoints(Pbot, P1) > distancePoints(Pbot, P2))
    {
      return P2;
    } else {
      return P1;
    }
}

cv::Point SearchTarget::findPointsAtDistanceX2(const cv::Point& A, const cv::Point& B, double d, const cv::Point& Pbot) {
    // Calculate the midpoint between A and B
    double mid_x = (A.x + B.x) / 2;
    double mid_y = (A.y + B.y) / 2;

    double m{0.0};
    if ((B.x - A.x)!=0)
    {
      m = - (B.y - A.y) / (B.x - A.x);
    }

    double q = mid_y - m * mid_x;
    double b = 2 * (mid_x - m * (mid_y - q));
    double c = std::pow(mid_x,2) - std::pow(d, 2) + std::pow(mid_y - q, 2);

    cv::Point P1{};
    cv::Point P2{};
    P1.x = (b + std::sqrt(std::pow(b, 2) - 4*c)) / 2;
    P2.x = (b - std::sqrt(std::pow(b, 2) - 4*c)) / 2;

    P1.y = m * P1.x + q;
    P2.y = m * P2.x + q;

    if (distancePoints(Pbot, P1) > distancePoints(Pbot, P2))
    {
      return P2;
    } else {
      return P1;
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

double SearchTarget::distancePoints(cv::Point P1, cv::Point P0) {
    return std::sqrt((P1.x - P0.x) * (P1.x - P0.x) + (P1.y - P0.y) * (P1.y - P0.y));
}

// Trova il punto nel set con la minima distanza dalla retta
void SearchTarget::findNearestPointToSegment(cv::Point& nearestPoint, double& min_distance, std::vector<cv::Point>& points, cv::Point P1, cv::Point P2) {
   min_distance = 10000000;
   // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
   double A = P2.y - P1.y;
   double B = P2.x - P1.x;
   double C = P2.x*P1.y - P2.y*P1.x;
   double den = std::sqrt(std::pow(B, 2) + std::pow(A, 2));
   for (const auto& point : points) {
        double distance = std::abs(A*point.x - B*point.y + C)/den;
        if (distance < min_distance) {
            min_distance = distance;
            nearestPoint = point;
        }
    }
   
}

void SearchTarget::closestPointOnSegment(cv::Point& cp, double& dist, cv::Point P0, cv::Point P1, cv::Point P2)
{
  cv::Point AB = P2 - P1;
  cv::Point AP = P0 - P1;
  double ab2 = AB.dot(AB);
  double ap_ab = AP.dot(AB);
  double t = ap_ab / ab2;

  // Clamp t to the range [0, 1]
  t = std::max(0.0, std::min(1.0, t));

  // Compute the closest point
  cp = P1 + AB * t;

  // Calculate the distance between P and the closest point
  dist = distancePoints(cp, P0);
}

void SearchTarget::findNearestPointToSegment2(cv::Point& nearestPoint, double& min_distance, std::vector<cv::Point>& points, cv::Point P1, cv::Point P2) {
   min_distance = std::numeric_limits<double>::max();
   // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
   double distance;
   cv::Point cp;
   for (const auto& point : points) {
        closestPointOnSegment(cp, distance, point, P1, P2);
        if (distance < min_distance) {
            min_distance = distance;
            nearestPoint = point;
        }
    }   
}

void SearchTarget::findNearestPointToLine(cv::Point& nearestPoint, double& min_distance, std::vector<cv::Point>& points, cv::Point P1, cv::Point P2) {
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
    min_distance = 10000000;
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