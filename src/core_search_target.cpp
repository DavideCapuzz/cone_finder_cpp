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
#include "cone_finder_cpp/core_search_target.hpp"

std::tuple<bool, geometry_msgs::msg::Point> CoreSearchTarget::update(
    Point &bot, OccupancyGrid &oc, BotOdom &odom, SearchTargetParams& params)
{
  bool found{false};
  geometry_msgs::msg::Point p_target{};
  cv::Mat map_image = oc.walls_.to_image();

  // find the closet point, 1 right, -1 left
  Point p_lateral = tools_.rotate_point_on_image(Point(bot.x_,bot.y_ + params.search_dir_ * 500), bot, odom.rotation_angle_);
  // find point in front of the direction of the robot
  Point p_front = tools_.rotate_point_on_image(Point(bot.x_, bot.y_ + 5), bot, odom.rotation_angle_ - 90);
  if (oc.p_100_.size() > 0) // check if the cost map has some obstacles
  {
    // groups all the lines in contours
    // find the closest point of the costmap closest to the robot
    auto [closest_point, distance] = tools_.find_nearest_point_2_segment2(oc.p_100_, bot, p_lateral);
    std::vector<Point> line; // line of contnious point on the wall in the direction of the robot
    if (tools_.distance_points(bot, closest_point) > 0)
    {
      if (params.dev_mode_ > 0)
      {
        cv::line(map_image, bot.to_cvpoint(), p_lateral.to_cvpoint(), cv::Vec3b(255, 0, 0), 1); // blue
        cv::line(map_image, bot.to_cvpoint(), p_front.to_cvpoint(), cv::Vec3b(0, 0, 255), 1);   // red
        map_image.at<cv::Vec3b>(bot.to_cvpoint()) = cv::Vec3b(0, 250, 0);
        map_image.at<cv::Vec3b>(closest_point.to_cvpoint()) = cv::Vec3b(0, 0, 250);
        if (params.dev_mode_ > 2)
        {
          cv::imshow("Image window", common_.zoom_image(map_image, params.zoom_));
          cv::waitKey(0);
        }
      }
      bool result{false};
      Point next_p{closest_point};
      params.kernel_.set_angle(odom.rotation_angle_);
      double rot_angle_kernel{};
      Matrix<5> local_costm_map{};
      int iterations{0};
      if (params.dev_mode_ > 2)
      {
        std::cout << "starting matrix\n";
        params.kernel_.print_mat();
        if (params.dev_mode_ > 2)
        {
          cv::imshow("Image kernel", common_.zoom_image(params.kernel_.to_image(), params.zoom_));
          cv::waitKey(0);
        }
      }
      do
      {
        ++iterations;
        if (line.size() > 3)
        {
          // if line greater than 3 use the direction of the line otherwise use the direction of the robot
          if ((line[line.size() - 3].x_ - line[line.size() - 2].x_) != 0)
          {
            rot_angle_kernel = atan((line[line.size() - 3].y_ - line[line.size() - 2].y_) / (line[line.size() - 3].x_ - line[line.size() - 2].x_)) * 180 / 3.14;
          }
          else if ((line[line.size() - 3].y_ - line[line.size() - 2].y_) > 0)
          {
            rot_angle_kernel = 90;
          }
          else
          {
            rot_angle_kernel = -90;
          }
          if (params.dev_mode_ > 2)
          {
            std::cout << " rot_angle_kernel " << rot_angle_kernel << "\n";
          }
          params.kernel_.set_angle(rot_angle_kernel);
        }
        else
        {
          params.kernel_.set_angle(odom.rotation_angle_);
        }

        for (int j = -static_cast<int>(params.kernel_.size() / 2); j <= static_cast<int>(params.kernel_.size() / 2); j++)
        {
          for (int i = -static_cast<int>(params.kernel_.size() / 2); i <= static_cast<int>(params.kernel_.size() / 2); i++)
          {
            if (params.dev_mode_ > 2)
            {
              std::cout << "i " << static_cast<int>(params.kernel_.size() / 2) + i << " j " << static_cast<int>(params.kernel_.size() / 2) + j << " i " << next_p.x_ + i << " j " << next_p.y_ + j << " v " << oc.walls_(next_p + Point(i, j)) << "\n";
            }
            local_costm_map[static_cast<int>(params.kernel_.size() / 2) + i][static_cast<int>(params.kernel_.size() / 2) + j] =
                oc.walls_(next_p + Point(i, j));
          }
        }
        if (params.dev_mode_ > 3)
        {
          cv::imshow("Image window", common_.zoom_image(params.kernel_.print_kernel_on_image(map_image, next_p), params.zoom_));
          cv::waitKey(3);
        }
        auto [result_in, i_r, j_r] = params.kernel_.evalute_next_point(local_costm_map);
        result = result_in;
        next_p = next_p + Point(i_r - 2, j_r - 2);
        if (result)
        {
          // Point next_p;
          // std::vector<Point> line;
          std::vector<Point>::iterator it;
          it = std::find(line.begin(), line.end(), next_p);
          if (it != line.end())
          {
            result = false;
            if (params.dev_mode_ > 2)
            {
              std::cout << "Element found in myvector\n";
            }
          }
          else
          {
            if (params.dev_mode_ > 2)
            {
              std::cout << "Element not found in myvector\n";
            }
            line.push_back(next_p);
          }
        }

        if (params.dev_mode_ > 0)
        {
          map_image.at<cv::Vec3b>(next_p.to_cvpoint()) = cv::Vec3b(0, 250, 0);
        }
        if (params.dev_mode_ > 2)
        {
          std::cout << "it " << iterations << "\n";
        }
      } while (result && iterations < 1000);

      if (line.size() > 5)
      {
        int last_idx = line.size() - 1;
        Point p_t1 = line.back();
        Point p_t2 = line[last_idx - 2];

        // find the point that is at a distance x to the fartest point of the robot in the direction of the research
        Point p_t3 = tools_.find_points_at_distance_X2(p_t1, p_t2, params.distance_wall_, bot); // not working well need to fix
        p_target = tools_.map_2_position(p_t3.x_, p_t3.y_, oc.map_res_, oc.map_x0_, oc.map_y0_);
        found = true;
        if (params.dev_mode_ > 0)
        {
          cv::line(map_image, p_t1.to_cvpoint(), p_t3.to_cvpoint(), cv::Vec3b(255, 0, 255), 1); // purple
          if (params.dev_mode_ > 2)
          {
            map_image.at<cv::Vec3b>(p_t1.to_cvpoint()) = cv::Vec3b(0, 255, 255); // yellow
            map_image.at<cv::Vec3b>(p_t2.to_cvpoint()) = cv::Vec3b(0, 255, 0);   // green
            map_image.at<cv::Vec3b>(p_t3.to_cvpoint()) = cv::Vec3b(255, 255, 0); // liht blue
            if (params.dev_mode_ > 3)
            {
              cv::imshow("Image window", common_.zoom_image(map_image, params.zoom_));
              cv::waitKey(0);
            }
          }
        }
      }
    }
  }

  if (params.dev_mode_ > 0)
  {
    cv::imshow("Image window", common_.zoom_image(map_image, params.zoom_));
    if (params.dev_mode_ == 1)
    {
      cv::waitKey(3);
    }
    else if (params.dev_mode_ > 1)
    {
      cv::waitKey(0);
    }
  }
  return std::make_tuple(found, p_target);
}