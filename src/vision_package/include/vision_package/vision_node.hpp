#pragma once
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


using namespace std::chrono_literals;

class VisionNode : public rclcpp::Node {
public:
    VisionNode(const rclcpp::NodeOptions& options)
    : Node("vision_node", options)
    , published_msg_count_(1) {
        data_publisher_ = this->create_publisher<std_msgs::msg::String>("vision_data", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&VisionNode::on_timer_ringing, this));
    }

private:
    void on_timer_ringing();

private:
    uint32_t                                                published_msg_count_;

    rclcpp::Publisher<::std_msgs::msg::String>::SharedPtr   data_publisher_;
    rclcpp::TimerBase::SharedPtr                            timer_;
};
