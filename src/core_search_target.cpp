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
#include "cone_finder_cpp/tools.hpp"
#include "cone_finder_cpp/core_search_target.hpp"

geometry_msgs::msg::Point CoreSearchTarget::update(
    int r_bot, int c_bot, float rotation_angle, double distance_wall, int search_dir,
    double angle_offset, Kernel<5> &kernel, int dev_mode)
{
  geometry_msgs::msg::Point p_target{};
  // find camera position inside the costmap
  cv::Point bot_pos = cv::Point(r_bot, c_bot);
  // find the closet point, 1 right, -1 left
  cv::Point p_lateral = tools_.rotate_point_on_image(cv::Point(r_bot, search_dir * 500), bot_pos, rotation_angle);
  // find point in front of the direction of the robot
  cv::Point p_front = tools_.rotate_point_on_image(cv::Point(r_bot, c_bot + 5), bot_pos, rotation_angle - 90);
  if (oc_.p_100_.size() > 0) // check if the cost map has some obstacles
  {
    double distance{0};
    // groups all the lines in contours
    cv::Point closest_point;
    // find the closest point of the costmap closest to the robot
    tools_.find_nearest_point_2_segment2(closest_point, distance, oc_.p_100_, bot_pos, p_lateral);
    std::vector<cv::Point> line; // line of contnious point on the wall in the direction of the robot

    if (tools_.distance_points(bot_pos, closest_point) > 0)
    {
      if (dev_mode > 0)
      {
        oc_.im_occgrid_.at<cv::Vec3b>(bot_pos) = cv::Vec3b(250, 0, 0);
        oc_.im_occgrid_.at<cv::Vec3b>(closest_point) = cv::Vec3b(250, 0, 0);
        cv::imshow("Image window", common_.zoom_image(oc_.im_occgrid_, 20.0));
        cv::waitKey(0);
        cv::circle(oc_.im_occgrid_, closest_point, 2, cv::Vec3b(0, 0, 0), 1, cv::LINE_8); // black
        cv::imshow("Image window", common_.zoom_image(oc_.im_occgrid_, 20.0));
        cv::waitKey(0);
      }
      bool result{false};
      double next_i = closest_point.x;
      double next_j = closest_point.y;
      // core_.flip();     if (dev_mode > 1)
      // oc_.matrix_.matrix_(static_cast<int>(next_j), static_cast<int>(next_i))=255;
      if (dev_mode > 0)
      {
        std::cout<<"next_i "<<next_i<<"    next_j "<<next_j<<"\n";
        cv::Mat oh = oc_.matrix_.to_image();
        oh.at<cv::Vec3b>(cv::Point(static_cast<int>(next_i), static_cast<int>(next_j))) = cv::Vec3b(0, 255, 0);
          
        cv::imshow("Image window2", common_.zoom_image(oh, 20.0));
        cv::waitKey(0);
      }

      kernel.set_angle(rotation_angle);
      double r{};
      Matrix<5> c_m{};
      int iterations{0};
      if (dev_mode > 1)
      {
        std::cout << "starting matrix\n";
        kernel.print_mat();
      }
      do
      {
        ++iterations;
        if (line.size() > 3)
        {
          // if line greater than 3 use the direction of the line otherwise use the direction of the robot
          if ((line[line.size() - 3].x - line[line.size() - 2].x) != 0)
          {
            r = atan((line[line.size() - 3].y - line[line.size() - 2].y) / (line[line.size() - 3].x - line[line.size() - 2].x)) * 180 / 3.14;
          }
          else if ((line[line.size() - 3].y - line[line.size() - 2].y) > 0)
          {
            r = 90;
          }
          else
          {
            r = -90;
          }
          // std::cout<<" r "<<r<<" r1 "<<angle_offset -r<<"\n";
          kernel.set_angle(angle_offset + r);
        }
        else
        {
          kernel.set_angle(rotation_angle);
        }
        std::cout << "start for\n";
        for (int j = -2; j <= 2; j++)
        {
          for (int i = -2; i <= 2; i++)
          {
            std::cout << "i " << 2 + i << " j " << 2 + j << " i " << static_cast<int>(next_i) + i << " j " << static_cast<int>(next_j) + j
                      << " v " << oc_.matrix_.matrix_(static_cast<int>(next_i) + i, static_cast<int>(next_j) + j) << "\n";
            c_m[2 + i][2 + j] = oc_.matrix_.matrix_(static_cast<int>(next_i) + i, static_cast<int>(next_j) + j);
            oc_.im_occgrid_.at<cv::Vec3b>(
                cv::Point(static_cast<int>(next_i) + i, static_cast<int>(next_j) + j)) = cv::Vec3b(250 * c_m[2 + i][2 + j], 0, 0);
          }
        }

        //
        double i_r;
        double j_r;
        std::tie(result, i_r, j_r) = kernel.evalute_next_point(c_m);

        next_i = next_i + i_r - 2;
        next_j = next_j + j_r - 2;

        if (result)
        {
          std::vector<cv::Point>::iterator it;
          it = std::find(line.begin(), line.end(), cv::Point(next_i, next_j));
          if (it != line.end())
          {
            result = false;
            if (dev_mode > 2)
            {
              std::cout << "Element found in myvector\n";
            }
          }
          else
          {
            if (dev_mode > 2)
            {
              std::cout << "Element not found in myvector\n";
            }
            line.push_back(cv::Point(next_i, next_j));
          }
        }

        if (dev_mode > 0)
        {
          // oc_.im_occgrid_.at<cv::Vec3b>(cv::Point(next_i, next_j)) = cv::Vec3b(0, 250, 0);
        }
        if (dev_mode > 2)
        {
          std::cout << "it " << iterations << "\n";
        }
        if (dev_mode > 1)
        {
          std::cout << "print kernel " << iterations << "\n";
          kernel.print_mat();
          std::cout << "print cm matrix\n";
          c_m.printMatrix();
        }
      } while (result && iterations < 1000);
      if (dev_mode > 1)
      {
        std::cout << "print kernel\n";
        kernel.print_mat();
        std::cout << "print cm matrix\n";
        c_m.printMatrix();
      }

      if (line.size() > 5)
      {
        int last_idx = line.size() - 1;
        cv::Point p_t1 = line.back();
        ;
        // cv::Point p_t2 = (line[last_idx-1]+line[last_idx-2]+line[last_idx-3])/3;
        cv::Point p_t2 = line[last_idx - 2];

        // find the point that is at a distance x to the fartest point of the robot in the direction of the research
        cv::Point p_t3 = tools_.find_points_at_distance_X2(p_t1, p_t2, distance_wall, bot_pos); // not working well need to fix
        // translate the point from image to occupancy grid
        // cv::Point p_t = tools_.cvpoint_2_grid(p_t3, oc_.grid_.info.height);
        tools_.map_2_position(p_t3.x, p_t3.y, oc_.map_res_, oc_.map_x0_, oc_.map_y0_, p_target);
        if (dev_mode > 0)
        {
          cv::line(oc_.im_occgrid_, p_t1, p_t3, cv::Vec3b(255, 0, 255), 1); // purple
          if (dev_mode > 2)
          {
            cv::circle(oc_.im_occgrid_, p_t1, 2, cv::Vec3b(0, 255, 255), 1, cv::LINE_8); // yellow
            cv::circle(oc_.im_occgrid_, p_t2, 2, cv::Vec3b(0, 255, 0), 1, cv::LINE_8);   // green
            cv::circle(oc_.im_occgrid_, p_t3, 2, cv::Vec3b(255, 255, 0), 1, cv::LINE_8); // liht blue
          }
        }
      }
    }
  }

  if (dev_mode > 0)
  {
    cv::line(oc_.im_occgrid_, bot_pos, p_lateral, cv::Vec3b(255, 0, 0), 1); // blue
    cv::line(oc_.im_occgrid_, bot_pos, p_front, cv::Vec3b(0, 0, 255), 1);   // red

    cv::imshow("Image window", common_.zoom_image(oc_.im_occgrid_, 20.0));
    // cv::imshow("Image window", oc_.im_occgrid_);
    if (dev_mode == 1)
    {
      cv::waitKey(3);
    }
    else if (dev_mode > 1)
    {
      cv::waitKey(0);
    }
  }
  return p_target;
}