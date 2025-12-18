#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class Listener : public rclcpp::Node
{
public:
    Listener();

private:
    void topic_callback(const std_msgs::msg::Int32::ConstSharedPtr msg) const;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

