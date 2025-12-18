#include "simple_subscriber.hpp"

Listener::Listener()
    : Node("listener")
{
    // QoS 설정: Reliable, Keep Last 10 (Publisher와 동일하게 설정)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .reliable()
                   .durability_volatile();

    sub_ = create_subscription<std_msgs::msg::Int32>(
        "counter", qos, std::bind(&Listener::topic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: counter");
}

void Listener::topic_callback(const std_msgs::msg::Int32::ConstSharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: %d", msg->data);
}

