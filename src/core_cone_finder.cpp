#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "cone_finder_cpp/search_target.hpp"
#include <cv_bridge/cv_bridge.hpp>               // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp>                  // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include "common/common.hpp"
#include <cmath>
#include <boost/math/special_functions/round.hpp>
#include "cone_finder_cpp/tools.hpp"
#include "cone_finder_cpp/core_cone_finder.hpp"

void CoreConeFinder::set_params(int dev_mode, double zoom)
{
  params_.dev_mode_ = dev_mode;
  params_.zoom_ = zoom;
}

geometry_msgs::msg::Point
CoreConeFinder::update(
    cv::Mat &in_image,
    sensor_msgs::msg::CameraInfo &camera_info, Point &p_cam, OccupancyGrid &oc, BotOdom &odom,
    vision_msgs::msg::BoundingBox2DArray &BB_array)
{
  geometry_msgs::msg::Point p_target{};
  std::vector<cv::Point> cone_coords;
  std::vector<double> cone_x{};
  std::tie(cone_coords, cone_x, BB_array) = find_cone(in_image, camera_info);

  if (cone_x.size() > 0)
  {
    p_target = find_cone_pose(p_cam, cone_x, oc, odom);
  }

  return p_target; // return bool also and BB
}

geometry_msgs::msg::Point CoreConeFinder::find_cone_pose(
    Point &p_cam, std::vector<double> &cone_x, OccupancyGrid &oc, BotOdom &odom)
{
  // find camera position inside the costmap
  geometry_msgs::msg::Point p_target{};
  cv::Mat map_image = oc.walls_.to_image();

  if (params_.dev_mode_ > 5)
  {
    std::cout << "p_cam.x  " << p_cam.x_ << " c_bot " << p_cam.y_ << " rot " << odom.rotation_angle_ << "\n";
  }
  Point p_front = tools_.rotate_point_on_image(Point(p_cam.x_, p_cam.y_ + 500), p_cam, odom.rotation_angle_ - 90);
  // get the angle of the camera position and rotate the point for the line
  std::vector<Point> p_nearest;
  if (oc.p_100_.size() > 0)
  {
    if (params_.dev_mode_ > 0)
    {
      cv::line(map_image, p_cam.to_cvpoint(), p_front.to_cvpoint(), cv::Vec3b(0, 0, 255), 1); // red
      map_image.at<cv::Vec3b>(p_cam.to_cvpoint()) = cv::Vec3b(0, 250, 0);
      if (params_.dev_mode_ > 2)
      {
        cv::imshow("Image window", common_.zoom_image(map_image, params_.zoom_));
        cv::waitKey(0);
      }
    }
    // for (double cone : cone_x)
    // {
    //   if (params_.dev_mode_ > 5)
    //   {
    //     std::cout << "cone x  " << cone << "\n";
    //   }
    //   Point p_t = tools_.rotate_point_on_image(Point(p_cam.x_ + cone * 500, 500.0), p_cam, odom.rotation_angle_ - 90);
    //   Point p_2 = tools_.rotate_point_on_image(Point(p_cam.x_ + cone * 500, 500.0), p_cam, -90);
    //   // cv::line (oc.occgrid_, p_cam,p_t, cv::Vec3b(255, 255,0), 1); // LIGHT BLUE
    //   // cv::line (oc.occgrid_, p_cam,p_2, cv::Vec3b(255,0,0), 1); // BLUE
    //   // cv::Point p = tools_.find_nearest_point(oc.p_100_, p_cam, p_t);
    //   // cv::Point p = tools_.find_nearest_point(oc.p_100_, p_cam, cv::Point(p_front.x * cone + p_front.x, 500));
    //   // p_nearest.push_back(cv::Point(p.x, p.y));
    // }
    // p_target = tools_.map_2_position(p_nearest[0].x_, p_nearest[0].y_, oc.map_res_, oc.map_x0_, oc.map_y0_);
  }

  // if (params_.dev_mode_ > 0)
  // {
  //   cv::namedWindow("map window");
  //   // cv::circle (oc.occgrid_, p_nearest[0], 4, cv::Vec3b(0, 255, 0), 4, cv::LINE_8); // green
  //   // cv::line (oc.occgrid_, p_cam, p_front, cv::Vec3b(255,0,255), 1); // PINK
  //   // cv::line (oc.occgrid_, p_cam, p_1, cv::Vec3b(0,0,255), 1); // RED
  //   // cv::line (oc.occgrid_, p_cam,cv::Point(p_front.x*cone_x[0]+p_front.x,500), cv::Vec3b(0,0,255), 1);
  //   // cv::imshow("map window", oc.occgrid_);

  //   if (params_.dev_mode_ > 5)
  //   {
  //     cv::waitKey(0);
  //   }
  //   else
  //   {
  //     cv::waitKey(3);
  //   }
  // }
  return p_target;
}

std::tuple<std::vector<cv::Point>, std::vector<double>, vision_msgs::msg::BoundingBox2DArray>
CoreConeFinder::find_cone(cv::Mat &in_image, sensor_msgs::msg::CameraInfo &camera_info)
{
  std::vector<cv::Point> cone_coords{};
  std::vector<double> cone_x{};
  vision_msgs::msg::BoundingBox2DArray BB_array{};
  // https://github.com/MicrocontrollersAndMore/Traffic_Cone_Detection_Visual_Basic/blob/master/frmMain.vb
  cv::Mat fullImageHSV;
  cv::Mat low_threshold;
  cv::Mat high_threshold;
  cv::cvtColor(in_image, fullImageHSV, CV_BGR2HSV);

  inRange(fullImageHSV, cv::Scalar(0, 130, 130), cv::Scalar(45, 255, 255), low_threshold);
  inRange(fullImageHSV, cv::Scalar(150, 135, 135), cv::Scalar(200, 255, 255), high_threshold);

  cv::Mat or_im;
  cv::bitwise_or(low_threshold, high_threshold, or_im);
  cv::Mat er_im;
  int erosion_size = 0;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                              cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                              cv::Point(erosion_size, erosion_size));

  cv::erode(or_im, er_im, element);
  cv::Mat dil_im;
  cv::erode(er_im, dil_im, element);
  cv::Mat gaus_im;
  cv::GaussianBlur(dil_im, gaus_im, cv::Size(3, 3), 0, 0);

  cv::Mat canny_im;
  cv::Canny(gaus_im, canny_im, 160, 80);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(canny_im, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Point> poly;
  std::vector<cv::Point> hull;
  std::vector<std::vector<cv::Point>> hull_refined;
  if (params_.dev_mode_ > 5)
  {
    std::cout << "contour size " << contours.size() << "\n";
  }
  for (size_t i = 0; i < contours.size(); ++i)
  {
    // approx poly need to be tune
    if (contours[i].size() > 20)
    {
      cv::approxPolyDP(contours[i], poly, 20, true);
    }
    else
    {
      poly = contours[i];
    }
    cv::convexHull(poly, hull);
    if (hull.size() >= 3)
    {
      if (pointingUp(hull))
      {
        hull_refined.push_back(hull);
      }
    }
  }
  cone_coords.clear();
  cone_x.clear();
  // TODO UNDERSTAND IF WE CAN SPLIT
  for (size_t i = 0; i < hull_refined.size(); i++)
  {
    if (params_.dev_mode_ > 5)
    {
      cv::drawContours(in_image, contours, (int)i, CV_RGB(255, 255, 0), 2, cv::LINE_8, hierarchy, 0);
      cv::drawContours(in_image, hull_refined, (int)i, CV_RGB(255, 0, 255));
    }

    cv::Moments m = cv::moments(hull_refined[i], true);
    cv::Point p(m.m10 / m.m00, m.m01 / m.m00);
    cv::Rect boundRect = cv::boundingRect(hull_refined[i]);
    double x = (p.x - camera_info.k[2]) / camera_info.k[0];
    if (params_.dev_mode_ > 5)
    {
      cv::circle(in_image, p, 10, CV_RGB(255, 0, 0));
      cv::rectangle(in_image, boundRect.tl(), boundRect.br(), CV_RGB(0, 0, 255), 2);
      std::cout << "p.x " << p.x << " camera_info.k[2] " << camera_info.k[2] << " camera_info.k[0] " << camera_info.k[0] << " x " << x << "\n";
    }
    cone_coords.push_back(p);
    cone_x.push_back(x);
    vision_msgs::msg::BoundingBox2D BB;
    BB.center.position.x = p.x;
    BB.center.position.y = p.y;
    BB.size_x = boundRect.width;
    BB.size_y = boundRect.height;
    BB_array.boxes.push_back(BB);
  }
  if (params_.dev_mode_ > 5)
  {
    cv::namedWindow("cone window");
    cv::imshow("cone window", in_image);
    cv::waitKey(0);
  }
  return std::make_tuple(cone_coords, cone_x, BB_array);
}

bool CoreConeFinder::pointingUp(std::vector<cv::Point> &hull_in)
{
  // evaluate aspect ratio
  cv::Rect boundRect = cv::boundingRect(hull_in);
  double aspect_ratio = boundRect.width / boundRect.height;

  if (aspect_ratio > 0.8)
  {
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

  for (cv::Point p : listOfPointsBelowCenter)
  {
    if (p.x < intLeftMostPointBelowCenter)
    {
      intLeftMostPointBelowCenter = p.x;
    }
  }

  for (cv::Point p : listOfPointsBelowCenter)
  {
    if (p.x < intRightMostPointBelowCenter)
    {
      intRightMostPointBelowCenter = p.x;
    }
  }

  for (cv::Point p : listOfPointsAboveCenter)
  {
    if (p.x < intLeftMostPointBelowCenter || p.x > intRightMostPointBelowCenter)
    {
      return false;
    }
  }

  return true;
}