#include <vector>
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp>                  // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <string>
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
#include <yaml-cpp/yaml.h>

#include <boost/math/special_functions/round.hpp>
#include <Eigen/Dense>
#include <tuple>
// #include <boost/numeric/ublas/matrix.hpp>
// #include <boost/numeric/ublas/io.hpp>

#ifndef ToolsCam_H
#define ToolsCam_H

struct Point
{
    Point(
        int x,
        int y) : x_(x),
                 y_(y)
    {
    }

    Point(
        double x,
        double y) : x_(static_cast<int>(x + 0.5)),
                    y_(static_cast<int>(y + 0.5))
    {
    }

    Point() : x_(0), y_(0)
    {
    }

    Point operator+(const Point &other) const
    {
        return Point(this->x_ + other.x_, this->y_ + other.y_);
    }
    Point operator-(const Point &other) const
    {
        return Point(this->x_ - other.x_, this->y_ - other.y_);
    }
    Point operator*(double other) const
    {
        return Point(this->x_ * other, this->y_ * other);
    }
    bool operator==(const Point &other) const
    {
        return this->x_ == other.x_ && this->y_ == other.y_;
    }
    double dot(const Point &other) const
    {
        return this->x_ * other.x_ + this->y_ * other.y_;
    }
    int row()
    {
        return y_;
    }
    int col()
    {
        return x_;
    }
    cv::Point to_cvpoint(){
        return cv::Point(x_, y_);
    }

    int x_;
    int y_;
};

class ToolsCam
{
public:
    ToolsCam();
    ~ToolsCam();

    Point position_2_map(
        geometry_msgs::msg::Point p, float map_res, float map_x0, float map_y0);

    geometry_msgs::msg::Point map_2_position(int ix, int iy, float map_res, float map_x0,  float map_y0);

    cv::Point rotate_point_on_image(
        const cv::Point &given_pt, const cv::Point &ref_pt, const double &angle_deg);
    Point rotate_point_on_image(
        const Point &given_pt, const Point &ref_pt, const double &angle_deg);

    cv::Point find_nearest_point(
        std::vector<cv::Point> &points, cv::Point P1, cv::Point P2);

    void find_nearest_point_2_line(
        cv::Point &nearestPoint, double &min_distance, std::vector<cv::Point> &points, cv::Point P1, cv::Point P2);
    void find_nearest_point_2_segment(
        cv::Point &nearestPoint, double &min_distance, std::vector<cv::Point> &points, cv::Point P1, cv::Point P2);
    std::tuple<cv::Point, double> find_nearest_point_2_segment2(
        std::vector<cv::Point> &points, cv::Point P1, cv::Point P2);
    std::tuple<Point, double> find_nearest_point_2_segment2(
        std::vector<Point> &points, Point P1, Point P2);
    std::tuple<cv::Point, double>  closest_point_on_segment(
        cv::Point P0, cv::Point P1, cv::Point P2);
    std::tuple<Point, double> closest_point_on_segment(
        Point P0, Point P1, Point P2);
    double distance_points(
        cv::Point P1, cv::Point P0);
    double distance_points(
        Point P1, Point P0);
    cv::Point find_points_at_distance_X(
        const cv::Point &A, const cv::Point &B, double x, const cv::Point &Pbot);
    cv::Point find_points_at_distance_X2(
        const cv::Point &A, const cv::Point &B, double d, const cv::Point &Pbot);
    Point find_points_at_distance_X2(
        const Point &A, const Point &B, double d, const Point &Pbot);

    void grid_2_image(
        nav_msgs::msg::OccupancyGrid &occupancyGrid, std::vector<cv::Point> &p_100, cv::Mat &c_mat_image, cv::Mat &c_mat_walls);

    Point grid_2_point(int grid_index, size_t grid_width, size_t grid_height);
    cv::Point grid_2_cvpoint(int grid_index, size_t grid_width, size_t grid_height);
    double distance_point_2_line(
        cv::Point P, double m, double q);

    cv::Point cvpoint_2_grid(cv::Point p, size_t grid_height);

    bool WriteMapToImage(std::string &name, nav_msgs::msg::OccupancyGrid &map);
};

template <int Size>
class Matrix
{
    using Row = std::array<double, Size>;
    using Matrix_template = std::array<Row, Size>;
    Matrix_template matrix_;

public:
    Matrix(Matrix_template &&matrix)
        : matrix_{std::move(matrix)} {}

    Matrix()
        : matrix_{} {}

    Row &operator[](std::size_t idx) { return matrix_[idx]; }
    const Row &operator[](std::size_t idx) const { return matrix_[idx]; }

    void printMatrix() const
    {
        for (auto const &row : matrix_)
        {
            for (auto const val : row)
            {
                std::cout << val << "\t";
            }
            std::cout << '\n';
        }
    }
};

template <int Size>
class Kernel
{
private:
    Matrix<Size> kernel_;
    Matrix<Size> kernel_original_;

    int class_;
    bool debug{false};
    static constexpr double fraction_{45.0};

public:
    Kernel(Matrix<Size> &&matrix)
        : kernel_{std::move(matrix)},
          kernel_original_{kernel_}
    {
        if (debug)
        {
            std::cout << "class " << class_ << '\n';
        }
    }
    Kernel()
        : kernel_{},
          kernel_original_{kernel_}
    {
        if (debug)
        {
            std::cout << "class " << class_ << '\n';
        }
    }

    void set_kernel(Matrix<Size> &&matrix){
        kernel_ = matrix;
        kernel_original_ = matrix;
    }

    void print_mat() const { kernel_.printMatrix(); }
    void print_original_mat() const { kernel_original_.printMatrix(); }
    void print_original_angle() const { kernel_original_.printAngle(); }

    void set_angle(double ang)
    {
        if (get_class(ang) != class_)
        {
            class_ = get_class(ang);
            if (debug)
            {
                std::cout << "new class " << class_ << '\n';
            }
            shift(class_);
        }
        else
        {
            if (debug)
            {
                std::cout << "old class " << class_ << '\n';
            }
        }
    }

    void flip()
    {
        for (int i = 0; i < Size; i++)
        {
            for (int j = 0; j < (Size - 1) / 2; j++)
            {
                double a = kernel_original_[i][j];
                kernel_original_[i][j] = kernel_original_[i][Size - 1 - j];
                kernel_original_[i][Size - j - 1] = a;
            }
        }
    }

    auto get_class(double ang)
    {
        return static_cast<int>(fmod(ang, 360.0 - 22.5) / fraction_);
    }

    std::tuple<bool, int, int> evalute_next_point(const Matrix<Size> &matrix)
    {
        double max_value{0.0};
        bool found{false};
        int i_out{0};
        int j_out{0};
        for (int i = 0; i < Size; i++)
        {
            for (int j = 0; j < Size; j++)
            {
                if (kernel_[i][j] * matrix[i][j] > max_value)
                {
                    max_value = kernel_[i][j] * matrix[i][j];
                    i_out = i;
                    j_out = j;
                    found = true;
                }
            }
        }

        if (!found)
        {
            i_out = (Size - 1) / 2;
            j_out = (Size - 1) / 2;
        }
        // Packing values to return a tuple
        return std::make_tuple(found, i_out, j_out);
    }

    void shift(int shift)
    {
        std::vector<double> a;
        for (int i = 0; i < static_cast<int>(Size / 2); i++)
        {
            if (debug)
            {
                std::cout << i << '\n';
            }
            if (debug)
            {
                std::cout << "a\n";
            }
            for (int j = i; j < Size - i; j++)
            {
                a.push_back(kernel_original_[i][j]);
                if (debug)
                {
                    std::cout << i << "\t" << j << "\t"
                              << kernel_original_[i][j] << '\n';
                }
            }
            if (debug)
            {
                std::cout << "b\n";
            }
            for (int j = i + 1; j < Size - i - 1; j++)
            {
                a.push_back(kernel_original_[j][Size - i - 1]);
                if (debug)
                {
                    std::cout << j << "\t" << Size - i - 1 << "\t"
                              << kernel_original_[j][Size - i - 1] << '\n';
                }
            }
            if (debug)
            {
                std::cout << "c\n";
            }
            for (int j = Size - i - 1; j >= i; j--)
            {
                a.push_back(kernel_original_[Size - i - 1][j]);
                if (debug)
                {
                    std::cout << Size - i - 1 << "\t" << j << "\t"
                              << kernel_original_[Size - i - 1][j] << '\n';
                }
            }
            if (debug)
            {
                std::cout << "d\n";
            }
            for (int j = Size - i - 2; j > i; j--)
            {
                a.push_back(kernel_original_[j][i]);
                if (debug)
                {
                    std::cout << j << "\t" << i << "\t"
                              << kernel_original_[j][i] << '\n';
                }
            }

            int s = static_cast<int>(fmod((Size / 2 - i) * shift + a.size(), a.size()));
            if (debug)
            {
                std::cout << "oh1 " << s << "\n";
            }
            for (int j = 0; j < s; j++)
            {
                double el = a.back();
                a.insert(a.begin(), el);
                a.pop_back();
                if (debug)
                {
                    std::cout << el << '\n';
                }
            }
            if (debug)
            {
                std::cout << "a\n";
            }
            for (int j = i; j < Size - i; j++)
            {
                kernel_[i][j] = a.front();
                a.erase(a.begin());
                if (debug)
                {
                    std::cout << i << "\t" << j << "\t" << kernel_[i][j]
                              << '\n';
                }
            }
            if (debug)
            {
                std::cout << "b\n";
            }
            for (int j = i + 1; j < Size - i - 1; j++)
            {
                kernel_[j][Size - i - 1] = a.front();
                a.erase(a.begin());
                if (debug)
                {
                    std::cout << j << "\t" << Size - i - 1 << "\t"
                              << kernel_[j][Size - i - 1] << '\n';
                }
            }
            if (debug)
            {
                std::cout << "c\n";
            }
            for (int j = Size - i - 1; j >= i; j--)
            {
                kernel_[Size - i - 1][j] = a.front();
                a.erase(a.begin());
                if (debug)
                {
                    std::cout << Size - i - 1 << "\t" << j << "\t"
                              << kernel_[Size - i - 1][j] << '\n';
                }
            }
            if (debug)
            {
                std::cout << "d\n";
            }
            for (int j = Size - i - 2; j > i; j--)
            {
                kernel_[j][i] = a.front();
                a.erase(a.begin());
                if (debug)
                {
                    std::cout << j << "\t" << i << "\t" << kernel_[j][i]
                              << '\n';
                }
            }
        }
    }

    cv::Mat to_imagegray(){
        cv::Mat gray_image(cv::Mat::ones(Size, Size, CV_8UC1) * 255);
        for (int i = 0; i < Size; i++)
        {
            for (int j = 0; j < Size; j++)
            {
                gray_image.at<uchar>(cv::Point(i,j)) = kernel_[i][j] * 255;
            }
        };
        return gray_image;
    }
    cv::Mat to_image(){
        cv::Mat color_image;
        cv::cvtColor(to_imagegray(), color_image, cv::COLOR_GRAY2BGR);
        return color_image;
    }
    cv::Mat print_kernel_on_image(cv::Mat image, Point center){
        // Assuming kernel.to_image() returns a cv::Mat
        cv::Mat kernel_image = to_image();

        // Define the region of interest (ROI) in image
        cv::Rect roi(center.x_ - static_cast<int>(Size / 2), center.y_ - static_cast<int>(Size / 2), kernel_image.cols, kernel_image.rows);

        // Ensure the ROI is within the bounds of image
        if (roi.x >= 0 && roi.y >= 0 && roi.x + roi.width <= image.cols && roi.y + roi.height <= image.rows)
        {
          // Copy the kernel_image to the specified ROI in image
          kernel_image.copyTo(image(roi));
        }
        else
        {
          std::cerr << "ROI is out of bounds!" << std::endl;
        }
        return image;
    }
    int size(){
        return Size;
    }

};

class BotOdom
{
public:
    // Constructor that initializes the pose and computes the rotation angle from the odometry data
    BotOdom(const nav_msgs::msg::Odometry &odom)
    {
        pose_ = odom.pose.pose;
        // rotation_angle_ = 2 * acos(pose_.orientation.w) * 180 / 3.14;
        std::tie(rotation_angle_, std::ignore, std::ignore) = quaternionToRPYAngleDegrees(odom.pose.pose.orientation);
    }

    BotOdom() : pose_{}, rotation_angle_{} {}

    geometry_msgs::msg::Pose pose_; // The pose of the bot
    double rotation_angle_;         // The rotation angle in degrees

    std::tuple<double, double, double> quaternionToRPYAngle(geometry_msgs::msg::Quaternion q)
    {
        double t0 = +2.0 * (q.w * q.x + q.y * q.z);
        double t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        double roll_x = atan2(t0, t1);

        double t2 = +2.0 * (q.w * q.y - q.z * q.x);
        t2 = std::fmin(1.0, std::fmax(-1.0, t2)); // Clamp to the range [-1, 1]
        double pitch_y = asin(t2);

        double t3 = +2.0 * (q.w * q.z + q.x * q.y);
        double t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double yaw_z = atan2(t3, t4);

        return std::make_tuple(roll_x * 180 / M_PI, pitch_y * 180 / M_PI, yaw_z * 180 / M_PI);
    }

    std::tuple<double, double, double> quaternionToRPYAngleDegrees(geometry_msgs::msg::Quaternion q)
    {
        auto[roll_x, pitch_y, yaw_z] =quaternionToRPYAngle(q);

        return std::make_tuple(roll_x * 180 / M_PI, pitch_y * 180 / M_PI, yaw_z * 180 / M_PI);
    }
};

class OccupancyGridMatrix
{
public:
    // Constructor with nav_msgs::msg::OccupancyGrid as input and height/width as parameters
    // https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html
    OccupancyGridMatrix(int height, int width, nav_msgs::msg::OccupancyGrid occupancyGrid)
        : height_(height), width_(width), matrix_(width, height) // Initialize matrix
    {
        std::cout << "height  " << height << "       width  " << width << "\n";
        for (int i = 0; i < width * height; ++i)
        {
            int col = static_cast<int>(i / width);
            int row = static_cast<int>(i % width);
            if (occupancyGrid.data[i] == 100)
            {
                matrix_(row, col) = 100;
            }
            else
            {
                matrix_(row, col) = 0;
            }
        }
        std::cout << "conversion done\n";
    }
    void print()
    {
        for (int i = 0; i < height_; ++i)
        {
            for (int j = 0; j < width_; ++j)
            {
                std::cout << matrix_(j, i) << "\t";
            }
            std::cout << "\n";
        }
    }
    cv::Mat to_image()
    {
        cv::Mat image(cv::Mat::zeros(height_, width_, CV_8UC3));
        for (int i = 0; i < height_; ++i)
        {
            for (int j = 0; j < width_; ++j)
            {
                // color = cv::Vec3b(std::min(255, color[0]), std::min(255, color[1]), std::min(255, color[2]));
                image.at<cv::Vec3b>(cv::Point(j, i)) = cv::Vec3b(250 * matrix_(j, i) / 100, 0, 0);
            }
        }
        return image;
    }

    // Default constructor
    OccupancyGridMatrix(int height, int width) : height_(height), width_(width), matrix_(width, height) {}

    // Destructor
    ~OccupancyGridMatrix() {}

    int height_;             // Store height
    int width_;              // Store width
    Eigen::MatrixXd matrix_; // Matrix with size (width, height)
};

class Grid
{
public:
    // Constructor with width and height
    Grid(int width, int height) : width_(width), height_(height), grid_(height_, width_)
    {
        // std::cout << grid_.cols() << " " << grid_.rows() << "\n";
    }

    // Default constructor
    Grid() : width_(0), height_(0), grid_() {}

    // Overload the operator() to access elements (const version)
    double operator()(int x, int y) const
    {
        return grid_(y, x);
    }
    double operator()(Point p) const
    {
        return grid_(p.y_, p.x_);
    }

    // Overload the operator() to modify elements
    double &operator()(int x, int y)
    {
        return grid_(y, x);
    }

    cv::Mat to_image()
    {
        cv::Mat image(cv::Mat::zeros(height_, width_, CV_8UC3));
        for (int i = 0; i < height_; ++i)
        {
            for (int j = 0; j < width_; ++j)
            {
                // color = cv::Vec3b(std::min(255, color[0]), std::min(255, color[1]), std::min(255, color[2]));
                image.at<cv::Vec3b>(i,j) = cv::Vec3b(250* grid_(i, j) /100, 0* grid_(i, j) /100, 0* grid_(i, j) /100);
            }
        }
        return image;
    }

    void print()
    {
        for (int i = 0; i < height_; ++i)
        {
            for (int j = 0; j < width_; ++j)
            {
                std::cout << grid_(i, j) << "\t";
            }
            std::cout << "\n";
        }
    }

private:
    int width_;
    int height_;
    Eigen::MatrixXd grid_;
};

class OccupancyGrid
{
public:
    // Constructor with nav_msgs::msg::OccupancyGrid as input
    OccupancyGrid(nav_msgs::msg::OccupancyGrid occupancyGrid)
        : map_res_(occupancyGrid.info.resolution),
          map_x0_(occupancyGrid.info.origin.position.x),
          map_y0_(occupancyGrid.info.origin.position.y),
          height_(occupancyGrid.info.height),
          width_(occupancyGrid.info.width),
          grid_(occupancyGrid),
          occgrid_(width_, height_),
          walls_(width_, height_)
    {
        // Convert from occupancy grid to image
        for (int i{0}; i < width_ * height_; ++i)
        {
            if (grid_.data[i] != 0)
            {
                Point p = tools_.grid_2_point(i, width_, height_);
                occgrid_(p.x_, p.y_) = grid_.data[i];
                if (grid_.data[i] == 100)
                {
                    walls_(p.x_, p.y_) = grid_.data[i];
                    p_100_.push_back(p);
                }
                else {
                    walls_(p.x_, p.y_) =0;
                }
            }
        }
    }

    // Default constructor
    OccupancyGrid() : map_res_(0), map_x0_(0), map_y0_(0), occgrid_(0, 0), walls_(0, 0)
    {
    }

    // Destructor
    ~OccupancyGrid() {}

    ToolsCam tools_{};
    float map_res_{};
    float map_x0_{};
    float map_y0_{};
    int height_{};
    int width_{};
    nav_msgs::msg::OccupancyGrid grid_{};
    Grid occgrid_{};
    Grid walls_{};
    std::vector<Point> p_100_{}; // Vector of the walls points
    // OccupancyGridMatrix matrix_; // Matrix representing the occupancy grid
};

#endif
