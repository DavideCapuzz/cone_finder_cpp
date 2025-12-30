#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "cone_finder_cpp/tools.hpp"
#include <cv_bridge/cv_bridge.hpp>               // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp>                  // We include everything about OpenCV as we don't care much about compilation time at the moment.

#include <cmath>
#include <boost/math/special_functions/round.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

ToolsCam::ToolsCam() {}

ToolsCam::~ToolsCam() {}

void ToolsCam::grid_2_image(nav_msgs::msg::OccupancyGrid &occupancyGrid, std::vector<cv::Point> &p_100, cv::Mat &c_mat_image, cv::Mat &c_mat_walls)
{
  // extract from the message cost map the r-c of a opencv image
  c_mat_image.setTo(cv::Scalar(255, 255, 255));
  c_mat_walls.setTo(cv::Scalar(255, 255, 255));

  for (u_int i{0}; i < occupancyGrid.info.width * occupancyGrid.info.height; ++i)
  {
    if (occupancyGrid.data[i] != 0)
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
}

cv::Point ToolsCam::grid_2_cvpoint(int grid_index, size_t grid_width, size_t grid_height)
{
  // convert occupacy grid point to image
  // + 0.5 for automatic rounding
  return cv::Point(static_cast<int>(grid_index % grid_width + 0.5), grid_height -1 -static_cast<int>(grid_index / grid_width + 0.5));
}

Point ToolsCam::grid_2_point(int grid_index, size_t grid_width, size_t grid_height)
{
  // convert occupacy grid point to image
  // + 0.5 for automatic rounding
  // row major order matrix so we start from the top position
  return Point(static_cast<int>(grid_index % grid_width + 0.5), grid_height -1 - static_cast<int>(grid_index / grid_width + 0.5));
}

cv::Point ToolsCam::cvpoint_2_grid(cv::Point p, size_t grid_height)
{
  // convert p image to r c
  // TO VERIFY IF IT IS CORRECT
  return cv::Point(p.x, p.y);
}

cv::Point ToolsCam::find_points_at_distance_X(const cv::Point &A, const cv::Point &B, double x, const cv::Point &Pbot)
{
  // Calculate the distance between A and B
  double d = distance_points(A, B);

  // Check if the distance is possible
  if (d > 2 * x)
  {
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
  cv::Point P1 = {(int)(mid_x + h * vx + 0.5), (int)(mid_y + h * vy + 0.5)};
  cv::Point P2 = {(int)(mid_x - h * vx + 0.5), (int)(mid_y - h * vy + 0.5)};

  if (distance_points(Pbot, P1) > distance_points(Pbot, P2))
  {
    return P2;
  }
  else
  {
    return P1;
  }
}

cv::Point ToolsCam::find_points_at_distance_X2(const cv::Point &A, const cv::Point &B, double d, const cv::Point &Pbot)
{
  // Calculate the midpoint between A and B
  // double mid_x = (A.x + B.x) / 2;
  // double mid_y = (A.y + B.y) / 2;
  cv::Point P1{};
  cv::Point P2{};

  double m{0.0};
  if ((B.x - A.x) != 0)
  {
    m = -(B.y - A.y) / (B.x - A.x);
    if (m != 0)
    {
      double q = A.y - m * A.x;
      double k = q - A.y;
      double a = 1 + std::pow(m, 2);

      double b = 2 * (-A.x + m * k);
      double c = std::pow(A.x, 2) - std::pow(d, 2) + std::pow(k, 2);

      P1.x = (-b + std::sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
      P2.x = (-b - std::sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
      P1.y = m * P1.x + q;
      P2.y = m * P2.x + q;
      // std::cout << m << "  " << q << "  " << k << "  " << a << "  " << b << " " << c << " " << "\n";
    }
    else
    {
      P1.x = A.x;
      P2.x = A.x;
      P1.y = A.y + d;
      P2.y = A.y - d;
    }
  }
  else
  {
    P1.x = A.x + d;
    P2.x = A.x - d;
    P1.y = A.y;
    P2.y = A.y;
  }
  // std::cout<<A.x<<"  "<<A.y<<"  "<<B.x<<" "<<B.y<<" "<<"\n";
  // std::cout<<P1.x<<"  "<<P1.y<<"  "<<P2.x<<" "<<P2.y<<" "<<distance_points(Pbot, P1) <<" "<<distance_points(Pbot, P2) <<"\n";

  if (distance_points(Pbot, P1) > distance_points(Pbot, P2))
  {
    return P2;
  }
  else
  {
    return P1;
  }
  /*// Calculate the midpoint between A and B
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
  }*/
}

Point ToolsCam::find_points_at_distance_X2(const Point &A, const Point &B, double d, const Point &Pbot)
{
  // Calculate the midpoint between A and B
  // double mid_x = (A.x + B.x) / 2;
  // double mid_y = (A.y_ + B.y_) / 2;
  Point P1{};
  Point P2{};

  double m{0.0};
  if ((B.x_ - A.x_) != 0)
  {
    m = -(B.y_ - A.y_) / (B.x_ - A.x_);
    if (m != 0)
    {
      double q = A.y_ - m * A.x_;
      double k = q - A.y_;
      double a = 1 + std::pow(m, 2);

      double b = 2 * (-A.x_ + m * k);
      double c = std::pow(A.x_, 2) - std::pow(d, 2) + std::pow(k, 2);

      P1.x_ = (-b + std::sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
      P2.x_ = (-b - std::sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
      P1.y_ = m * P1.x_ + q;
      P2.y_ = m * P2.x_ + q;
      // std::cout << m << "  " << q << "  " << k << "  " << a << "  " << b << " " << c << " " << "\n";
    }
    else
    {
      P1.x_ = A.x_;
      P2.x_ = A.x_;
      P1.y_ = A.y_ + d;
      P2.y_ = A.y_ - d;
    }
  }
  else
  {
    P1.x_ = A.x_ + d;
    P2.x_ = A.x_ - d;
    P1.y_ = A.y_;
    P2.y_ = A.y_;
  }

  if (distance_points(Pbot, P1) > distance_points(Pbot, P2))
  {
    return P2;
  }
  else
  {
    return P1;
  }
}

cv::Point ToolsCam::rotate_point_on_image(const cv::Point &given_pt, const cv::Point &ref_pt, const double &angle_deg)
{
  double rotation_angle = angle_deg * M_PI / 180.0;
  cv::Point rotated_pt;

  rotated_pt.x = (given_pt.x - ref_pt.x) * cos(rotation_angle) - (given_pt.y - ref_pt.y) * sin(rotation_angle) + ref_pt.x;
  rotated_pt.y = (given_pt.x - ref_pt.x) * sin(rotation_angle) + (given_pt.y - ref_pt.y) * cos(rotation_angle) + ref_pt.y;

  return rotated_pt;
}

Point ToolsCam::rotate_point_on_image(const Point &given_pt, const Point &ref_pt, const double &angle_deg)
{
  double rotation_angle = angle_deg * M_PI / 180.0;
  Point rotated_pt{};

  rotated_pt.x_ = (given_pt.x_ - ref_pt.x_) * cos(rotation_angle) - (given_pt.y_ - ref_pt.y_) * sin(rotation_angle) + ref_pt.x_;
  rotated_pt.y_ = (given_pt.x_ - ref_pt.x_) * sin(rotation_angle) + (given_pt.y_ - ref_pt.y_) * cos(rotation_angle) + ref_pt.y_;

  return rotated_pt;
}

double ToolsCam::distance_point_2_line(cv::Point P, double m, double q)
{
  return std::abs(m * P.x - P.y + q) / std::sqrt(m * m + 1);
}

double ToolsCam::distance_points(cv::Point P1, cv::Point P0)
{
  return std::sqrt((P1.x - P0.x) * (P1.x - P0.x) + (P1.y - P0.y) * (P1.y - P0.y));
}
double ToolsCam::distance_points(Point P1, Point P0)
{
  return std::sqrt((P1.x_ - P0.x_) * (P1.x_ - P0.x_) + (P1.y_ - P0.y_) * (P1.y_ - P0.y_));
}

// Trova il punto nel set con la minima distanza dalla retta
void ToolsCam::find_nearest_point_2_segment(cv::Point &nearestPoint, double &min_distance, std::vector<cv::Point> &points, cv::Point P1, cv::Point P2)
{
  min_distance = std::numeric_limits<double>::max();
  // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
  double A = P2.y - P1.y;
  double B = P2.x - P1.x;
  double C = P2.x * P1.y - P2.y * P1.x;
  double den = std::sqrt(std::pow(B, 2) + std::pow(A, 2));
  for (const auto &point : points)
  {
    double distance = std::abs(A * point.x - B * point.y + C) / den;
    if (distance < min_distance)
    {
      min_distance = distance;
      nearestPoint = point;
    }
  }
}

std::tuple<cv::Point, double> ToolsCam::closest_point_on_segment(
    cv::Point P0, cv::Point P1, cv::Point P2)
{
  cv::Point AB = P2 - P1;
  cv::Point AP = P0 - P1;
  double ab2 = AB.dot(AB);
  double ap_ab = AP.dot(AB);
  double t = ap_ab / ab2;

  // Clamp t to the range [0, 1]
  t = std::max(0.0, std::min(1.0, t));

  // Compute the closest point
  cv::Point cp = P1 + AB * t;

  // Calculate the distance between P and the closest point
  double dist = distance_points(cp, P0);
  return std::make_tuple(cp, dist);
}

std::tuple<Point, double> ToolsCam::closest_point_on_segment(
    Point P0, Point P1, Point P2)
{
  Point AB = P2 - P1;
  Point AP = P0 - P1;
  double ab2 = AB.dot(AB);
  double ap_ab = AP.dot(AB);
  double t = ap_ab / ab2;

  // Clamp t to the range [0, 1]
  t = std::max(0.0, std::min(1.0, t));

  // Compute the closest point
  Point cp = P1 + AB * t;

  // Calculate the distance between P and the closest point
  double dist = distance_points(cp, P0);
  return std::make_tuple(cp, dist);
}

std::tuple<cv::Point, double> ToolsCam::find_nearest_point_2_segment2(
    std::vector<cv::Point> &points, cv::Point P1, cv::Point P2)
{
  cv::Point nearestPoint{};
  double min_distance = std::numeric_limits<double>::max();
  // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
  // double distance;
  // cv::Point cp;
  for (const auto &point : points)
  {
    auto [cp, distance] = closest_point_on_segment(point, P1, P2);
    if (distance < min_distance)
    {
      min_distance = distance;
      nearestPoint = point;
    }
  }
  return std::make_tuple(nearestPoint, min_distance);
}

std::tuple<Point, double> ToolsCam::find_nearest_point_2_segment2(
    std::vector<Point> &points, Point P1, Point P2)
{
  Point nearestPoint{};
  double min_distance = std::numeric_limits<double>::max();
  // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
  double distance;
  Point cp;
  for (const auto &point : points)
  {
    auto [cp, distance] = closest_point_on_segment(point, P1, P2);
    if (distance < min_distance)
    {
      min_distance = distance;
      nearestPoint = point;
    }
  }
  return std::make_tuple(nearestPoint, min_distance);
}

void ToolsCam::find_nearest_point_2_line(cv::Point &nearestPoint, double &min_distance, std::vector<cv::Point> &points, cv::Point P1, cv::Point P2)
{
  // Calcola la pendenza della retta
  double m{0.0};
  if ((P2.x - P1.x) != 0)
  {
    m = (P2.y - P1.y) / (P2.x - P1.x);
  }
  // Calcola l'intercetta y della retta
  double q = P1.y - m * P1.x;
  // Inizializza la distanza minima e il punto corrispondente
  // double minDistance = std::numeric_limits<double>::max();
  // Scansiona tutti i punti nel set e trova il punto con la minima distanza
  min_distance = std::numeric_limits<double>::max();
  for (const auto &point : points)
  {
    double distance = distance_point_2_line(point, m, q);
    if (distance < min_distance)
    {
      min_distance = distance;
      nearestPoint = point;
    }
  }
}

cv::Point ToolsCam::find_nearest_point(std::vector<cv::Point> &points, cv::Point P1, cv::Point P2)
{
  // Evaluate slope
  double m{0.0};
  if ((P2.x - P1.x) != 0)
  {
    m = (P2.y - P1.y) / (P2.x - P1.x);
  }
  // Evaluate q
  double q = P1.y - m * P1.x;
  // Inizializza la distanza minima e il punto corrispondente
  double minDistance = std::numeric_limits<double>::max();
  cv::Point nearestPoint;
  // Scansiona tutti i punti nel set e trova il punto con la minima distanza
  for (const auto &point : points)
  {
    double distance = distance_point_2_line(point, m, q);
    if (distance < minDistance)
    {
      minDistance = distance;
      nearestPoint = point;
    }
    if (distance < 1)
    {
      return nearestPoint;
    }
  }

  return nearestPoint;
}

Point ToolsCam::position_2_map(geometry_msgs::msg::Point p, float map_res, float map_x0, float map_y0)
{
  // translate from xy to r c
  int ix, iy;
  ix = static_cast<int>(round((p.x - map_x0) / map_res));
  iy = static_cast<int>(round((p.y - map_y0) / map_res));
  return Point{ix, iy};
}

geometry_msgs::msg::Point ToolsCam::map_2_position(int ix, int iy, float map_res, float map_x0, float map_y0)
{
  // translate from xy to r c
  geometry_msgs::msg::Point p;
  p.x = (ix * map_res) + map_x0;
  p.y = (iy * map_res) + map_y0;
  p.z = 0;
  return p;
}

bool ToolsCam::WriteMapToImage(std::string &name,
                               nav_msgs::msg::OccupancyGrid &map)
{
  cv::Mat c_mat_image = cv::Mat::zeros(
      map.info.height, map.info.width, CV_8UC3);
  cv::Mat c_mat_walls = cv::Mat::zeros(
      map.info.height, map.info.width, CV_8UC3);
  cv::Mat c_mat_walls_gray = cv::Mat::zeros(
      map.info.height, map.info.width, CV_8UC3);

  std::vector<cv::Point> p_100; // vector of the walls points
  // convert from occupancy grid to image
  grid_2_image(map, p_100, c_mat_image, c_mat_walls);
  cv::imwrite(name, c_mat_walls); // A JPG FILE IS BEING SAVED
  std::cout << "[INFO] [map_io]: Map saved" << std::endl;
  return true;
}
