#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "cone_finder_cpp/tools.hpp"
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
#include <cmath>
#include <boost/math/special_functions/round.hpp>


using namespace std::chrono_literals;

ToolsCam::ToolsCam(){}

ToolsCam::~ToolsCam(){}


void ToolsCam::grid_2_image(nav_msgs::msg::OccupancyGrid &occupancyGrid, std::vector<cv::Point> &p_100, cv::Mat &c_mat_image, cv::Mat &c_mat_walls)
{
  // extract from the message cost map the r-c of a opencv image
  c_mat_image.setTo(cv::Scalar(255, 255, 255));
  c_mat_walls.setTo(cv::Scalar(255, 255, 255));
 
  for (u_int i{0}; i < occupancyGrid.info.width * occupancyGrid.info.height; ++i)
  {
    if(occupancyGrid.data[i] != 0)
    {
      cv::Point p = grid_2_cvpoint(i, occupancyGrid.info.width, occupancyGrid.info.height);
      if (occupancyGrid.data[i] == 100)
      {
        c_mat_image.at<cv::Vec3b>(p) = cv::Vec3b(0, 0, 255);
        c_mat_walls.at<cv::Vec3b>(p) = cv::Vec3b(0, 0, 0);            
        p_100.push_back(p);
      }
      else
      {
        // c_mat_image.at<cv::Vec3b>(p)=cv::Vec3b(0,255,0);
      }
    }
  }
}\

cv::Point ToolsCam::grid_2_cvpoint(int grid_index, size_t grid_width, size_t grid_height)
{
  // convert occupacy grid point to image
  // + 0.5 for automatic rounding
  return cv::Point(static_cast<int>(grid_index % grid_width + 0.5), static_cast<int>(grid_index / grid_height + 0.5));
}

cv::Point ToolsCam::cvpoint_2_grid(cv::Point p, size_t grid_height)
{
  // convert p image to r c
  // TO VERIFY IF IT IS CORRECT
  return  cv::Point(p.x, p.y);
}

cv::Point ToolsCam::find_points_at_distance_X(const cv::Point& A, const cv::Point& B, double x, const cv::Point& Pbot) {
  // Calculate the distance between A and B
  double d = distance_points(A, B);

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
  cv::Point P1 = {(int)(mid_x + h * vx + 0.5) , (int)(mid_y + h * vy + 0.5)};
  cv::Point P2 = { (int)(mid_x - h * vx + 0.5), (int)(mid_y - h * vy + 0.5)};

  if (distance_points(Pbot, P1) > distance_points(Pbot, P2))
  {
    return P2;
  } else {
    return P1;
  }
}

cv::Point ToolsCam::find_points_at_distance_X2(const cv::Point& A, const cv::Point& B, double d, const cv::Point& Pbot) {
    // Calculate the midpoint between A and B
    double mid_x = (A.x + B.x) / 2;
    double mid_y = (A.y + B.y) / 2;
    cv::Point P1{};
    cv::Point P2{};

    double m{0.0};
    if ((B.x - A.x)!=0)
    {
      
      m = - (B.y - A.y) / (B.x - A.x);
      if (m!=0)
      {
        double q = mid_y - m * mid_x;
        double b = 2 * (mid_x - m * (mid_y - q));
        double c = std::pow(mid_x,2) - std::pow(d, 2) + std::pow(mid_y - q, 2);
        
        P1.x = (b + std::sqrt(std::pow(b, 2) - 4*c)) / 2;
        P2.x = (b - std::sqrt(std::pow(b, 2) - 4*c)) / 2;
        P1.y = m * P1.x + q;
        P2.y = m * P2.x + q;
      }
      else{
        P1.x = mid_x;
        P2.x = mid_x;
        P1.y = mid_y+d;
        P2.y = mid_y-d;
      }
      
    }
    else{
      P1.x = mid_x +d ;
      P2.x = mid_x -d;
      P1.y = mid_y;
      P2.y = mid_y;
    }    

    if (distance_points(Pbot, P1) > distance_points(Pbot, P2))
    {
      return P2;
    } else {
      return P1;
    }
}


cv::Point ToolsCam::rotate_point_on_image(const cv::Point& given_pt, const cv::Point& ref_pt, const double& angle_deg) {
  double rotation_angle = angle_deg * M_PI / 180.0;
  cv::Point rotated_pt;

  rotated_pt.x = (given_pt.x - ref_pt.x) * cos(rotation_angle) - (given_pt.y - ref_pt.y) * sin(rotation_angle) + ref_pt.x;
  rotated_pt.y = (given_pt.x - ref_pt.x) * sin(rotation_angle) + (given_pt.y - ref_pt.y) * cos(rotation_angle) + ref_pt.y;

  return rotated_pt;
}

double ToolsCam::distance_point_2_line(cv::Point P, double m, double q) {
  return std::abs(m * P.x - P.y + q) / std::sqrt(m * m + 1);
}

double ToolsCam::distance_points(cv::Point P1, cv::Point P0) {
  return std::sqrt((P1.x - P0.x) * (P1.x - P0.x) + (P1.y - P0.y) * (P1.y - P0.y));
}

// Trova il punto nel set con la minima distanza dalla retta
void ToolsCam::find_nearest_point_2_segment(cv::Point& nearestPoint, double& min_distance, std::vector<cv::Point>& points, cv::Point P1, cv::Point P2) 
{
  min_distance = std::numeric_limits<double>::max();
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

void ToolsCam::closest_point_on_segment(cv::Point& cp, double& dist, cv::Point P0, cv::Point P1, cv::Point P2)
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
  dist = distance_points(cp, P0);
}

void ToolsCam::find_nearest_point_2_segment2(cv::Point& nearestPoint, double& min_distance, std::vector<cv::Point>& points, cv::Point P1, cv::Point P2) {
  min_distance = std::numeric_limits<double>::max();
  // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
  double distance;
  cv::Point cp;
  for (const auto& point : points) {
    closest_point_on_segment(cp, distance, point, P1, P2);
    if (distance < min_distance) {
      min_distance = distance;
      nearestPoint = point;
    }
  }   
}

void ToolsCam::find_nearest_point_2_line(cv::Point& nearestPoint, double& min_distance, std::vector<cv::Point>& points, cv::Point P1, cv::Point P2) {
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
  min_distance = std::numeric_limits<double>::max();
  for (const auto& point : points) {
    double distance = distance_point_2_line(point, m, q);
    if (distance < min_distance) {
      min_distance = distance;
      nearestPoint = point;
    }
  }
}

cv::Point ToolsCam::find_nearest_point(std::vector<cv::Point>& points, cv::Point P1, cv::Point P2) {
  // Evaluate slope
  double m{0.0};
  if ((P2.x - P1.x)!=0)
  {
    m = (P2.y - P1.y) / (P2.x - P1.x);
  }
  // Evaluate q
  double q = P1.y - m * P1.x;
  // Inizializza la distanza minima e il punto corrispondente
  double minDistance = std::numeric_limits<double>::max();
  cv::Point nearestPoint;
  // Scansiona tutti i punti nel set e trova il punto con la minima distanza
  for (const auto& point : points) {
    double distance = distance_point_2_line(point, m, q);
    if (distance < minDistance) {
      minDistance = distance;
      nearestPoint = point;
    }
    if (distance < 1) {
      return nearestPoint;
    }
  }

  return nearestPoint;
}


bool ToolsCam::test_time(rclcpp::Time (&msgs_time_)[4],rclcpp::Duration max_delta)
{
  // TODO function to test time
  (void) max_delta;/*
  rclcpp::Time t_ref = this->get_clock()->now();
  for (rclcpp::Time el : msgs_time_)
  {
    if ((t_ref - el)>max_delta){
      return false;
    }
    
  }*/
  return true;
}

void ToolsCam::position_2_map(geometry_msgs::msg::Point p, float map_res, float map_x0,  float map_y0, int& ix, int& iy)
{
  // translate from xy to r c 
	ix = static_cast<int>(round ((p.x - map_x0) / map_res));
	iy = static_cast<int>(round ((p.y - map_y0) / map_res));
}

void ToolsCam::map_2_position(int ix, int iy, float map_res, float map_x0,  float map_y0, geometry_msgs::msg::Point& p)
{
  // translate from xy to r c 
	p.x = (ix * map_res) + map_x0;
  p.y = (iy * map_res) + map_y0;
  p.z = 0;
}
