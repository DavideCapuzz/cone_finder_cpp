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

  service_ = this->create_service<interfaces::srv::GetTarget>(
    "/get_trg_fnd_pos", std::bind(&SearchTarget::get_search_target_server, this, _1, _2));

  publisher_maker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/target_pos_array", 10);
        
  // parameters
  this->declare_parameter("dev_mode_", 0);
  this->declare_parameter("distance_wall_", 0.5);
  this->declare_parameter("search_dir_", 1);

  dev_mode_ = this->get_parameter("dev_mode_").as_int();
  distance_wall_ = this->get_parameter("distance_wall_").as_double();
  search_dir_ = this->get_parameter("search_dir_").as_int();
  RCLCPP_INFO(this->get_logger(), "dev_mode  %d",dev_mode_ );

  // intialize all
  if (dev_mode_ > 0)
  {
    cv::namedWindow("Image window"); 
  }  

  RCLCPP_INFO(this->get_logger(), "start search target");
}

SearchTarget::~SearchTarget(){
  if (dev_mode_ > 0)
  {
    cv::destroyWindow("Image window");
  }
}

void SearchTarget::costmap_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in)
{
  nav_msgs::msg::OccupancyGrid occupancyGrid = * msg_in;

  // get map resolution and and position
  map_res_ = occupancyGrid.info.resolution;
  map_x0_ = occupancyGrid.info.origin.position.x;
  map_y0_ = occupancyGrid.info.origin.position.y;

  // check if the messages is update
  msgs_time_[2] = this->get_clock()->now();
  if (true)  // TODO: check if all the information are updated
  {
    // create image occupancy grid gray scale to fine countours
    cv::Mat c_mat_image = cv::Mat::zeros(
      occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);
    cv::Mat c_mat_walls = cv::Mat::zeros(
      occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);    
    cv::Mat c_mat_walls_gray = cv::Mat::zeros(
      occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);

    std::vector<cv::Point> p_100;  // vector of the walls points

    // convert from occupancy grid to image 
    tools_.grid_2_image(occupancyGrid, p_100, c_mat_image, c_mat_walls); 
    // find camera position inside the costmap     
    cv::Point bot_pos = cv::Point(r_bot_, c_bot_);
    // find the closet point, 1 right, -1 left
    cv::Point p_lateral = tools_.rotate_point_on_image(cv::Point(r_bot_, search_dir_ * 500), bot_pos, rotation_angle_);
    // se a front point of the direction of the robot
    cv::Point p_front = tools_.rotate_point_on_image(cv::Point(r_bot_, c_bot_ + 5), bot_pos, rotation_angle_- 90); 
    if (dev_mode_ > 0)
    {
      RCLCPP_INFO(this->get_logger(), "%d, %d %d %d ", bot_pos.x, bot_pos.y, p_lateral.x, p_lateral.y);
    }
    // get the angle of the camera position and rotate the point for the line
    std::vector<cv::Point> closest_line;
    cv::Point closest_point;
    // convert from rgb to gray scale
    cv::cvtColor(c_mat_walls, c_mat_walls_gray, cv::COLOR_BGR2GRAY);
    if (p_100.size() > 0) // check if the cost map has some obstacles
    {
      std::vector<std::vector<cv::Point>> contours;                   // array of lines
      std::vector<cv::Vec4i> hierarchy;                               // put param of the find contour algorithm
      int i{0};                                                       // index iterator
      int idx_cont{-1};                                               // index of the losest line
      double min_distance_line{std::numeric_limits<double>::max()};   // min distance of the line to the directional line
      double min_distance_robot{std::numeric_limits<double>::max()};  // min distance from the line to the robt

      // groups all the lines in contours
      cv::findContours(c_mat_walls_gray, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
      cv::Point closest_point;  
      for (std::vector<cv::Point> singel_line : contours)             // iterate over the array of contours
      {
        cv::Point cp;                                                 // closest point  
        double distance;                                              // distance of the line
        tools_.find_nearest_point_2_segment2(cp, distance, singel_line, bot_pos, p_lateral);
        if (dev_mode_ > 1)
        {
          RCLCPP_INFO(this->get_logger(), "%d, %d %d %f ", i, cp.x, cp.y, distance);
        }
        double d_robot = tools_.distance_points(cp, bot_pos);         // get distance between robot and the closest point of the line
        if ((min_distance_line > distance) && (min_distance_robot > d_robot) && (cp.x != 0 && cp.y != 0) && (singel_line.size() !=0))
        {
          // keep the closest line only if the closest point of the line is the closest to the robot compare to the other segments
          closest_line = singel_line;
          min_distance_line = distance;
          idx_cont = i;
          closest_point = cp;
        }
        ++i;
      }
      
      if (idx_cont>=0)
      {
        closest_line = contours[idx_cont];
        if (dev_mode_ > 0)
        {
          RCLCPP_INFO(this->get_logger(), "indx line  %d", idx_cont);
          RCLCPP_INFO(this->get_logger(), "%ld", closest_line.size());
        }
        if (dev_mode_ > 0)
        {
          cv::drawContours( c_mat_image, contours, idx_cont, cv::Vec3b(250,0,0), 1, cv::LINE_8, 0, 0); // blue
        } 

        // translate the direction vector to the closest point
        cv::Point delta = bot_pos - closest_point;
        cv::Point dir_line = p_front - delta;

        RCLCPP_INFO(this->get_logger(), "bpx %d, bpy %d, cpx %d, cpy %d, dx %d, dy %d, pfx %d, pfy %d, dlx %d, dly %d", 
          bot_pos.x, bot_pos.y, closest_point.x, closest_point.y,
          delta.x, delta.y, p_front.x, p_front.y,dir_line.x, dir_line.y
          );
        
        // find the closest point near to the dir_line inside the line and get the index
        // in one single iteration get also the index of front and back point of the line
        double m_dist = std::numeric_limits<double>::max();
        int i_x =0;
        int index_dir = 0;
        int index_first =0;
        int index_last =0;
        int closest_point_index = 0;
        for (cv::Point point : closest_line) 
        {
          RCLCPP_INFO(this->get_logger(), "i %d, px %d, py %d", 
          i_x , point.x, point.y);
          if (m_dist > tools_.distance_points(dir_line, point))
          {
            index_dir = i_x;
            m_dist = tools_.distance_points(dir_line, point);
          }
          if (point == closest_point)
          {
            closest_point_index = i_x;
          }
          if (dev_mode_ > 0)
          {
            if (point == closest_line.back())
            {
              index_first = i_x;
            }
            if (point == closest_line.front())
            {
              index_last = i_x;
            }
          }
          ++i_x;
        }
        if (dev_mode_ > 0)
        {
          RCLCPP_INFO(this->get_logger(), "size %ld, front %d, back %d, bot %d, dir %d", 
            closest_line.size(), index_last, index_first, closest_point_index, index_dir);
        }
        // decide if pick up the front point or the back point

        // iterate over all the other segment to check if there are closest segment
        // do while there are not closest segment in a radius 
        // every time remove the segments

        contours.erase(contours.begin() + idx_cont);
        for (std::vector<cv::Point> singel_line : contours) 
        {

        }
        
        // find the points that are close to the farthest point

        // evaluate the perpendicular point (already done)

        //
        
        int last_idx = closest_line.size() - 1;                           // last id of the line
        cv::Point far_point = closest_line.back();                        // point of the last id
        cv::Point p_t1{};                                                 // p1 used to evaluate the perpendicular distance
        cv::Point p_t2{};                                                 // p2 used to evaluate the perpendicular distance

        // select to use the last or the first points
        if (tools_.distance_points(p_front, far_point) < tools_.distance_points(bot_pos, far_point))
        {
          p_t1 = closest_line.front();
          p_t2 = (closest_line[1]+closest_line[2]+closest_line[3]+closest_line[4]+closest_line[5])/5;
          if (dev_mode_ > 0)
          {
            RCLCPP_INFO(this->get_logger(), "opt 1 %f %f %d %d %d %d ",
            tools_.distance_points(p_front, far_point) ,
            tools_.distance_points(bot_pos, far_point), 
            p_t1.x, p_t1.y,p_t2.x, p_t2.y);
          }
        } else {
          p_t1 = closest_line.back();;
          p_t2 = (closest_line[last_idx-1]+closest_line[last_idx-2]+closest_line[last_idx-3]+closest_line[last_idx-4]+closest_line[last_idx-5])/5;
          if (dev_mode_ > 0)
          {
            RCLCPP_INFO(this->get_logger(), "opt 2 %f %f %d %d %d %d ",
            tools_.distance_points(p_front, far_point) ,
            tools_.distance_points(bot_pos, far_point), 
            p_t1.x, p_t1.y,p_t2.x, p_t2.y);
          }
        }
      
      
      

        // find the point that is at a distance x to the fartest point of the robot in the direction of the research
        cv::Point p_t3 = tools_.find_points_at_distance_X2(p_t1, p_t2, distance_wall_, bot_pos); // not working well need to fix
        // translate the point from image to occupancy grid
        //cv::Point p_t = tools_.cvpoint_2_grid(p_t3, occupancyGrid.info.height);
        tools_.map_2_position(p_t3.x, p_t3.y, map_res_, map_x0_, map_y0_, p_target_);
        if (dev_mode_ > 0)
        {
          cv::line (c_mat_image, p_t1, p_t3, cv::Vec3b(255, 0, 255), 1); // purple
          cv::circle (c_mat_image, p_t1, 4, cv::Vec3b(0, 255, 255), 3, cv::LINE_8); // yellow
          cv::circle (c_mat_image, p_t2, 4, cv::Vec3b(0, 255, 0), 3, cv::LINE_8); // green
          cv::circle (c_mat_image, p_t3, 4, cv::Vec3b(255, 255, 0), 3, cv::LINE_8); // liht blue
        }
      }
    }
    
    std::vector<geometry_msgs::msg::Point> pp;
    pp.push_back(p_target_);
    publish_marker(pp);

    if (dev_mode_ > 0)
    {
      cv::line (c_mat_image, bot_pos, p_lateral, cv::Vec3b(255,0,0), 1); // blue
      cv::line (c_mat_image, bot_pos, p_front, cv::Vec3b(0,0,255), 1); // red
      
      cv::imshow("Image window", c_mat_walls_gray);
    
      cv::waitKey(3);
    }
    
  }
}

void SearchTarget::costmap_cb_old(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in)
{
  nav_msgs::msg::OccupancyGrid occupancyGrid = * msg_in;

  // get map resolution and and position
  map_res_ = occupancyGrid.info.resolution;
  map_x0_ = occupancyGrid.info.origin.position.x;
  map_y0_ = occupancyGrid.info.origin.position.y;

  // check if the messages is update
  msgs_time_[2] = this->get_clock()->now();
  if (true)  // TODO: check if all the information are updated
  {
    // create image occupancy grid gray scale to fine countours
    cv::Mat c_mat_image = cv::Mat::zeros(
      occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);
    cv::Mat c_mat_walls = cv::Mat::zeros(
      occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);    
    cv::Mat c_mat_walls_gray = cv::Mat::zeros(
      occupancyGrid.info.height, occupancyGrid.info.width, CV_8UC3);

    std::vector<cv::Point> p_100;  // vector of the walls points

    // convert from occupancy grid to image 
    tools_.grid_2_image(occupancyGrid, p_100, c_mat_image, c_mat_walls); 
    // find camera position inside the costmap     
    cv::Point bot_pos = cv::Point(r_bot_, c_bot_);
    // find the closet point, 1 right, -1 left
    cv::Point p_lateral = tools_.rotate_point_on_image(cv::Point(r_bot_, search_dir_ * 500), bot_pos, rotation_angle_);
    // se a front point of the direction of the robot
    cv::Point p_front = tools_.rotate_point_on_image(cv::Point(r_bot_, c_bot_ + 5), bot_pos, rotation_angle_- 90); 
    if (dev_mode_ > 0)
    {
      RCLCPP_INFO(this->get_logger(), "%d, %d %d %d ", bot_pos.x, bot_pos.y, p_lateral.x, p_lateral.y);
    }
    // get the angle of the camera position and rotate the point for the line
    std::vector<cv::Point> closest_line;
    cv::Point closest_point;
    // convert from rgb to gray scale
    cv::cvtColor(c_mat_walls, c_mat_walls_gray, cv::COLOR_BGR2GRAY);
    if (p_100.size() > 0) // check if the cost map has some obstacles
    {
      std::vector<std::vector<cv::Point>> contours;                   // array of lines
      std::vector<cv::Vec4i> hierarchy;                               // put param of the find contour algorithm
      int i{0};                                                       // index iterator
      int idx_cont{-1};                                               // index of the losest line
      double min_distance_line{std::numeric_limits<double>::max()};   // min distance of the line to the directional line
      double min_distance_robot{std::numeric_limits<double>::max()};  // min distance from the line to the robt

      // groups all the lines in contours
      cv::findContours(c_mat_walls_gray, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
      cv::Point closest_point;  
      for (std::vector<cv::Point> singel_line : contours)             // iterate over the array of contours
      {
        cv::Point cp;                                                 // closest point  
        double distance;                                              // distance of the line
        tools_.find_nearest_point_2_segment2(cp, distance, singel_line, bot_pos, p_lateral);
        if (dev_mode_ > 1)
        {
          RCLCPP_INFO(this->get_logger(), "%d, %d %d %f ", i, cp.x, cp.y, distance);
        }
        double d_robot = tools_.distance_points(cp, bot_pos);         // get distance between robot and the closest point of the line
        if ((min_distance_line > distance) && (min_distance_robot > d_robot) && (cp.x != 0 && cp.y != 0) && (singel_line.size() !=0))
        {
          // keep the closest line only if the closest point of the line is the closest to the robot compare to the other segments
          closest_line = singel_line;
          min_distance_line = distance;
          idx_cont = i;
          closest_point = cp;
        }
        ++i;
      }
      
      if (idx_cont>=0)
      {
        closest_line = contours[idx_cont];
        if (dev_mode_ > 0)
        {
          RCLCPP_INFO(this->get_logger(), "indx line  %d", idx_cont);
          RCLCPP_INFO(this->get_logger(), "%ld", closest_line.size());
        }
        if (dev_mode_ > 0)
        {
          cv::drawContours( c_mat_image, contours, idx_cont, cv::Vec3b(250,0,0), 1, cv::LINE_8, 0, 0); // blue
        } 

        // translate the direction vector to the closest point
        cv::Point delta = bot_pos - closest_point;
        cv::Point dir_line = p_front - delta;

        RCLCPP_INFO(this->get_logger(), "bpx %d, bpy %d, cpx %d, cpy %d, dx %d, dy %d, pfx %d, pfy %d, dlx %d, dly %d", 
          bot_pos.x, bot_pos.y, closest_point.x, closest_point.y,
          delta.x, delta.y, p_front.x, p_front.y,dir_line.x, dir_line.y
          );
        
        // find the closest point near to the dir_line inside the line and get the index
        // in one single iteration get also the index of front and back point of the line
        double m_dist = std::numeric_limits<double>::max();
        int i_x =0;
        int index_dir = 0;
        int index_first =0;
        int index_last =0;
        int closest_point_index = 0;
        for (cv::Point point : closest_line) 
        {
          RCLCPP_INFO(this->get_logger(), "i %d, px %d, py %d", 
          i_x , point.x, point.y);
          if (m_dist > tools_.distance_points(dir_line, point))
          {
            index_dir = i_x;
            m_dist = tools_.distance_points(dir_line, point);
          }
          if (point == closest_point)
          {
            closest_point_index = i_x;
          }
          if (dev_mode_ > 0)
          {
            if (point == closest_line.back())
            {
              index_first = i_x;
            }
            if (point == closest_line.front())
            {
              index_last = i_x;
            }
          }
          ++i_x;
        }
        if (dev_mode_ > 0)
        {
          RCLCPP_INFO(this->get_logger(), "size %ld, front %d, back %d, bot %d, dir %d", 
            closest_line.size(), index_last, index_first, closest_point_index, index_dir);
        }
        // decide if pick up the front point or the back point

        // iterate over all the other segment to check if there are closest segment
        // do while there are not closest segment in a radius 
        // every time remove the segments

        contours.erase(contours.begin() + idx_cont);
        for (std::vector<cv::Point> singel_line : contours) 
        {

        }
        
        // find the points that are close to the farthest point

        // evaluate the perpendicular point (already done)

        //
        
        int last_idx = closest_line.size() - 1;                           // last id of the line
        cv::Point far_point = closest_line.back();                        // point of the last id
        cv::Point p_t1{};                                                 // p1 used to evaluate the perpendicular distance
        cv::Point p_t2{};                                                 // p2 used to evaluate the perpendicular distance

        // select to use the last or the first points
        if (tools_.distance_points(p_front, far_point) < tools_.distance_points(bot_pos, far_point))
        {
          p_t1 = closest_line.front();
          p_t2 = (closest_line[1]+closest_line[2]+closest_line[3]+closest_line[4]+closest_line[5])/5;
          if (dev_mode_ > 0)
          {
            RCLCPP_INFO(this->get_logger(), "opt 1 %f %f %d %d %d %d ",
            tools_.distance_points(p_front, far_point) ,
            tools_.distance_points(bot_pos, far_point), 
            p_t1.x, p_t1.y,p_t2.x, p_t2.y);
          }
        } else {
          p_t1 = closest_line.back();;
          p_t2 = (closest_line[last_idx-1]+closest_line[last_idx-2]+closest_line[last_idx-3]+closest_line[last_idx-4]+closest_line[last_idx-5])/5;
          if (dev_mode_ > 0)
          {
            RCLCPP_INFO(this->get_logger(), "opt 2 %f %f %d %d %d %d ",
            tools_.distance_points(p_front, far_point) ,
            tools_.distance_points(bot_pos, far_point), 
            p_t1.x, p_t1.y,p_t2.x, p_t2.y);
          }
        }
      
      
      

        // find the point that is at a distance x to the fartest point of the robot in the direction of the research
        cv::Point p_t3 = tools_.find_points_at_distance_X2(p_t1, p_t2, distance_wall_, bot_pos); // not working well need to fix
        // translate the point from image to occupancy grid
        //cv::Point p_t = tools_.cvpoint_2_grid(p_t3, occupancyGrid.info.height);
        tools_.map_2_position(p_t3.x, p_t3.y, map_res_, map_x0_, map_y0_, p_target_);
        if (dev_mode_ > 0)
        {
          cv::line (c_mat_image, p_t1, p_t3, cv::Vec3b(255, 0, 255), 1); // purple
          cv::circle (c_mat_image, p_t1, 4, cv::Vec3b(0, 255, 255), 3, cv::LINE_8); // yellow
          cv::circle (c_mat_image, p_t2, 4, cv::Vec3b(0, 255, 0), 3, cv::LINE_8); // green
          cv::circle (c_mat_image, p_t3, 4, cv::Vec3b(255, 255, 0), 3, cv::LINE_8); // liht blue
        }
      }
    }
    
    std::vector<geometry_msgs::msg::Point> pp;
    pp.push_back(p_target_);
    publish_marker(pp);

    if (dev_mode_ > 0)
    {
      cv::line (c_mat_image, bot_pos, p_lateral, cv::Vec3b(255,0,0), 1); // blue
      cv::line (c_mat_image, bot_pos, p_front, cv::Vec3b(0,0,255), 1); // red
      
      cv::imshow("Image window", c_mat_image);
    
      cv::waitKey(3);
    }
    
  }
}



void SearchTarget::publish_marker(std::vector<geometry_msgs::msg::Point> & p_vector)
{
  //RCLCPP_INFO(this->get_logger(), " oh %f, %f" ,(p.x*map_res_+map_x0_),(p.y*map_res_+map_y0_));
  visualization_msgs::msg::MarkerArray marker_array;
  for (geometry_msgs::msg::Point p :p_vector)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "test_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = p;
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

void SearchTarget::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg_in)
{  
  msgs_time_[3] = this->get_clock()->now();
  bot_pose_.position = msg_in->pose.pose.position;
  bot_pose_.orientation = msg_in->pose.pose.orientation;
  // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
  geometry_msgs::msg::TransformStamped transformCamera;
  
  try{
    tools_.position_2_map(bot_pose_.position, map_res_, map_x0_, map_y0_, r_bot_, c_bot_);
    rotation_angle_ = 2 * acos(bot_pose_.orientation.w);
    
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Target finder: %s",ex.what());
    return;
  } 
}

void SearchTarget::get_search_target_server(const std::shared_ptr<interfaces::srv::GetTarget::Request> request,
          std::shared_ptr<interfaces::srv::GetTarget::Response>  response)
{
  (void) request;
  if (p_target_.x!=0 && p_target_.y!=0)
  {
    //RCLCPP_INFO(this->get_logger(), " founded !!");
    response->result = true;
    response->target = p_target_;    
    if (dev_mode_ > 0)
    {
      RCLCPP_INFO(this->get_logger(), "%f %f", p_target_.x * map_res_ + map_x0_, p_target_.y * map_res_ + map_y0_);
    }
  }
  else{
    //RCLCPP_INFO(this->get_logger(), " NOT founded !!");
    response->result=false;
    response->target.x=0;
    response->target.y=0;
  }
  response->type.data = "Target";
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SearchTarget>());
  rclcpp::shutdown();
  return 0;
}