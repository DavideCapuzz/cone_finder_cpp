#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cone_finder_cpp/cone_finder.h"
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 


using namespace std::chrono_literals;
using std::placeholders::_1;

ConeFinder::ConeFinder()
: Node("minimal_publisher"), count_(0)
{
    subscription_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&ConeFinder::imageCallBack, this, _1));
    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    // timer_ = this->create_wall_timer(500ms, std::bind(&ConeFinder::timer_callback, this));
}

ConeFinder::~ConeFinder(){}

void ConeFinder::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

void ConeFinder::imageCallBack(const sensor_msgs::msg::Image::SharedPtr msg_in)
{
    RCLCPP_INFO(this->get_logger(), " ho ricevuto !!");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg_in, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_INFO(this->get_logger(), " ho ricevuto !!");
      return;
    }
    /*
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());*/
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeFinder>());
  rclcpp::shutdown();
  return 0;
}