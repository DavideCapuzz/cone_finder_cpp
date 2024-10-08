#include <array>
#include <cmath>
#include <iostream>
#include <vector>

#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "interfaces/srv/get_target.hpp"

#include "cone_finder_cpp/tools.hpp"

#ifndef SearchTarget_H
#define SearchTarget_H

template <int Size>
class Matrix {
    using Row = std::array<double, Size>;
    using Matrix_template = std::array<Row, Size>;
    Matrix_template matrix_;

    public:
    Matrix(Matrix_template&& matrix)
        : matrix_{std::move(matrix)} {}

    Row& operator[](std::size_t idx) { return matrix_[idx]; }
    const Row& operator[](std::size_t idx) const { return matrix_[idx]; }

    void printMatrix() const {
        for (auto const& row : matrix_) {
            for (auto const val : row) {
                std::cout << val << "\t";
            }
            std::cout << '\n';
        }
    }
};

template <int Size>
class Kernel {
   private:
    Matrix<Size> kernel_;
    Matrix<Size> kernel_original_;

    int class_;
    bool debug{false};
    static constexpr double fraction_{45.0};

   public:
    Kernel(Matrix<Size>&& matrix)
        : kernel_{std::move(matrix)},
          kernel_original_{kernel_} {
        if (debug) {
            std::cout << "class " << class_ << '\n';
        }
    }

    void print_mat() const { kernel_.printMatrix(); }
    void print_original_mat() const { kernel_original_.printMatrix(); }
    void print_original_angle() const { kernel_original_.printAngle(); }

    void set_angle(double ang) {
        if (get_class(ang) != class_) {
            class_ = get_class(ang);
            shift(class_);
            if (debug) {
                std::cout << "new class " << class_ << '\n';
            }
        } else {
            if (debug) {
                std::cout << "old class " << class_ << '\n';
            }
        }
    }

    void flip()
    {
        for (int i = 0; i < (Size-1)/2; i++) {
            for (int j = 0; j < Size; j++) {
                double a = kernel_original_[i][j];
                kernel_original_[i][j] = kernel_original_[Size-i-1][j];
                kernel_original_[Size-i-1][j] = a;
            }
        }
    }

    auto get_class(double ang) {
        return static_cast<int>(fmod(ang, 360.0 - 22.5) / fraction_);
    }

    std::tuple<bool, int, int> evalute_next_point(const Matrix<Size>& matrix) {
        double max_value{0.0};
        bool found{false};
        int i_out{0};
        int j_out{0};
        for (int i = 0; i < Size; i++) {
            for (int j = 0; j < Size; j++) {
                if (kernel_[i][j] * matrix[i][j] > max_value) {
                    max_value = kernel_[i][j] * matrix[i][j];
                    i_out = i;
                    j_out = j;
                    found = true;
                }
            }
        }

        if (!found)
        {
            i_out = (Size -1)/2;
            j_out = (Size -1)/2;
        }
        // Packing values to return a tuple
        return std::make_tuple(found, i_out, j_out);
    }

    void shift(int shift) {
        std::vector<double> a;
        for (int i = 0; i < static_cast<int>(Size / 2); i++) {
            for (int j = i; j < Size - i; j++) {
                a.push_back(kernel_original_[i][j]);
                if (debug) {
                    std::cout << i << "\t" << j << "\t"
                              << kernel_original_[i][j] << '\n';
                }
            }
            for (int j = i + 1; j < Size - i - 1; j++) {
                a.push_back(kernel_original_[j][Size - i - 1]);
                if (debug) {
                    std::cout << j << "\t" << Size - i - 1 << "\t"
                              << kernel_original_[j][Size - i - 1] << '\n';
                }
            }
            for (int j = Size - i - 1; j >= i; j--) {
                a.push_back(kernel_original_[Size - i - 1][j]);
                if (debug) {
                    std::cout << Size - i - 1 << "\t" << j << "\t"
                              << kernel_original_[Size - i - 1][j] << '\n';
                }
            }
            for (int j = Size - i - 1; j > i; j--) {
                a.push_back(kernel_original_[j][i]);
                if (debug) {
                    std::cout << j << "\t" << i << "\t"
                              << kernel_original_[j][i] << '\n';
                }
            }
            if (debug) {
                std::cout << "oh1 "
                          << (static_cast<int>((Size) / 2) - i) * shift << '\n';
            }
            for (int j = 0; j < (static_cast<int>((Size) / 2) - i) * shift;
                 j++) {
                double el = a.back();
                a.insert(a.begin(), el);
                a.pop_back();
                if (debug) {
                    std::cout << el << '\n';
                }
            }
            if (debug) {
                std::cout << "oh2" << '\n';
            }
            for (int j = i; j < Size - i; j++) {
                kernel_[i][j] = a.front();
                a.erase(a.begin());
                if (debug) {
                    std::cout << i << "\t" << j << "\t" << kernel_[i][j]
                              << '\n';
                }
            }
            for (int j = i + 1; j < Size - i - 1; j++) {
                kernel_[j][Size - i - 1] = a.front();
                a.erase(a.begin());
                if (debug) {
                    std::cout << j << "\t" << Size - i - 1 << "\t"
                              << kernel_[j][Size - i - 1] << '\n';
                }
            }
            for (int j = Size - i - 1; j >= i; j--) {
                kernel_[Size - i - 1][j] = a.front();
                a.erase(a.begin());
                if (debug) {
                    std::cout << Size - i - 1 << "\t" << j << "\t"
                              << kernel_[Size - i - 1][j] << '\n';
                }
            }
            for (int j = Size - i - 1; j > i; j--) {
                kernel_[j][i] = a.front();
                a.erase(a.begin());
                if (debug) {
                    std::cout << j << "\t" << i << "\t" << kernel_[j][i]
                              << '\n';
                }
            }
        }
    }
};

class SearchTarget: public rclcpp::Node
{
public:
	SearchTarget();
	~SearchTarget();

private:
  // ros interface
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;

  rclcpp::Service<interfaces::srv::GetTarget>::SharedPtr service_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_maker_;
  rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr publisher_BB_;

  // input callback
  void costmap_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in);
  void costmap_cb_old(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in);
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg_in); 

  void get_search_target_server(const std::shared_ptr<interfaces::srv::GetTarget::Request> request,
          std::shared_ptr<interfaces::srv::GetTarget::Response> response);

  // internal functions
 void publish_marker(std::vector<geometry_msgs::msg::Point> & p_vector);

  // input
  float map_x0_{}, map_y0_{}, map_res_{};
  geometry_msgs::msg::Pose bot_pose_{}, goal_pose_{};
  
  
  int r_bot_{0};                  // row the base link of the robot
	int c_bot_{0};                  // column the base link of the robot
  float rotation_angle_{0.0};     // heading of the robot

  // internal data
  ToolsCam tools_{};              // tools cone finder node 
  rclcpp::Time msgs_time_[4];     // test to chech if all the messages has been updated

  // output
  geometry_msgs::msg::Point p_target_{};          // next target point

  // parameters 
  int dev_mode_{0};               // dev level 0 - 3
  double distance_wall_{10};      // parameter of the algorithm
  int search_dir_{1};             // 1 right, -1 left direction of research

//   Kernel<5> kernel_{Matrix<5>{
//     {0.0, 0.0, 0.25, 0.22, 0.31, 
//     0.0,  0.0, 0.3, 0.28, 0.32, 
//     0.0,  0.0, 0.0, 1, 0.7,
//     0.0,  0.0, 0.55, 0.45, 0.5,
//     0.0, 0.0,  0.65, 0.35, 0.4}}};

  Kernel<5> kernel_{Matrix<5>{
    {0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0,  0.0, 0.0, 0.0, 0.0, 
    0.0,  0.0, 0.0, 0.3, 0.25,
    0.45,  0.5, 1, 0.28, 0.32,
    0.4, 0.35,  0.7, 0.22, 0.31}}};

rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_visualize_image_;

};

#endif

