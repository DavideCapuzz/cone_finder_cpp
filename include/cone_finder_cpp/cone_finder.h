// Copyright 2019 coderkarl. Subject to the BSD license.

#ifndef ConeFinder_H
#define ConeFinder_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>


class ConeFinder: public rclcpp::Node
{
public:
	ConeFinder();
	~ConeFinder();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;
    void imageCallBack(const sensor_msgs::msg::Image::SharedPtr msg_in);
    size_t count_;
};

#endif
