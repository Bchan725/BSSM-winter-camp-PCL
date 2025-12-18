#include "simple_publisher.hpp"

Talker::Talker()
    : Node("talker"), count_(0)
{
    // QoS 설정: Reliable, Keep Last 10
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .reliable()
                   .durability_volatile();

    pub_ = create_publisher<std_msgs::msg::Int32>("counter", qos);

    timer_ = create_wall_timer(
        1s, [this]()
        {
            std_msgs::msg::Int32 msg;
            msg.data = count_++;

            RCLCPP_INFO(get_logger(), "publish: %d", msg.data);
            pub_->publish(msg);
        });
}
