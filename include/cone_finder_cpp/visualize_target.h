// Copyright 2019 coderkarl. Subject to the BSD license.

#ifndef VisualizeTarget_H
#define VisualizeTarget_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> 


class VisualizeTarget: public rclcpp::Node
{
public:
	VisualizeTarget();
	~VisualizeTarget();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_visualize_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;    
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_cam_info_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_info_;
    rclcpp::Subscription<vision_msgs::msg::BoundingBox2D>::SharedPtr sub_cone_;

    void image_CB(const sensor_msgs::msg::Image::SharedPtr msg_in);
    void imageInfo_CB(const sensor_msgs::msg::CameraInfo::SharedPtr msg_in);
    void coneBB_CB(const vision_msgs::msg::BoundingBox2D::SharedPtr msg_in);

    size_t count_;
    //sensor_msgs::msg::Image::SharedPtr msg_;
    cv::Mat img_canva_;
    cv_bridge::CvImagePtr ptr_share_;
    // static const std::string OPENCV_WINDOW = "Image window";
};

#endif
