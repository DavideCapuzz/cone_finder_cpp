#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cone_finder_cpp/visualize_target.h"
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.


using namespace std::chrono_literals;
using std::placeholders::_1;

VisualizeTarget::VisualizeTarget()
: Node("Visualize_target"), count_(0) , img_canva_(cv::Size(300, 300), CV_8UC3)
{
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&VisualizeTarget::image_CB, this, _1));    
    pub_visualize_image_ = this->create_publisher<sensor_msgs::msg::Image>("/target/cam_target", 10);
    sub_cam_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", 10, std::bind(&VisualizeTarget::imageInfo_CB, this, _1));
    pub_cam_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/target/camera_info", 10);

    sub_cone_ = this->create_subscription<vision_msgs::msg::BoundingBox2D>(
      "/cone/BB", 10, std::bind(&VisualizeTarget::coneBB_CB, this, _1));
    //timer_ = this->create_wall_timer(500ms, std::bind(&VisualizeTarget::timer_callback, this)); 
    cv::namedWindow("Image window");   
}

VisualizeTarget::~VisualizeTarget(){
  cv::destroyWindow("Image window");
}

void VisualizeTarget::timer_callback(){
  //cv::Mat img_canva_(cv::Size(300, 300), CV_8UC3);
    // Create a new 640x480 image 
    // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    sensor_msgs::msg::Image::SharedPtr msg;
    msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_canva_)
               .toImageMsg();
 
    // Publish the image to the topic defined in the publisher
    // RCLCPP_INFO(this->get_logger(), "Image1 %ld published", count_);
    pub_visualize_image_->publish(*msg.get());
    RCLCPP_INFO(this->get_logger(), "Image2 %ld published", count_);
    
}

void VisualizeTarget::imageInfo_CB(const sensor_msgs::msg::CameraInfo::SharedPtr msg_in)
{
    pub_cam_info_->publish(*msg_in.get());
}

void VisualizeTarget::coneBB_CB(const vision_msgs::msg::BoundingBox2D::SharedPtr msg_in)
{
    RCLCPP_INFO(this->get_logger(), "read target !!");
}

void VisualizeTarget::image_CB(const sensor_msgs::msg::Image::SharedPtr msg_in)
{
    // RCLCPP_INFO(this->get_logger(), " leggo");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg_in, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_INFO(this->get_logger(), " errore !!");
      return;
    }
    
    //RCLCPP_INFO(this->get_logger(), "Image width %d height %d", cv_ptr->image.cols,cv_ptr->image.rows );
    
    count_++;
    
    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    //img_canva_ = cv_ptr->image;

    sensor_msgs::msg::Image::SharedPtr msg;
    msg = cv_bridge::CvImage(msg_in->header, "bgr8", cv_ptr->image)
               .toImageMsg();
 
    // Publish the image to the topic defined in the publisher
    // RCLCPP_INFO(this->get_logger(), "Image1 %ld published", count_);
    // pub_visualize_image_->publish(cv_ptr->toImageMsg());
    try
    {
      pub_visualize_image_->publish(*msg);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_INFO(this->get_logger(), " errore !!");
      return;
    }

    
    //RCLCPP_INFO(this->get_logger(), "Image2 %ld published", count_);
    // Update GUI Window
    //cv::imshow("Image window", cv_ptr->image);
    //cv::waitKey(3);
    /*
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());*/
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizeTarget>());
  rclcpp::shutdown();
  return 0;
}