#pragma once
#include <chrono>

#include <rclcpp/rclcpp.hpp>

using std::chrono_literals;


class VisionNode : public rclcpp::Node {
public:
    VisionNode()
    : Node("vision_node")
    , publishing_count_(1) {
        data_publisher_ = this->create_publisher<std_msgs::msg::String>("vision_data");
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&VisionNode::on_timer_ringing, this));
    }

private:
    void on_timer_ringing() {
        RCLCPP_INFO(this->get_logger(), "Publishing data");
        auto msg = std_msgs::msg::String();
        msg.data = "Current status of vision: " + std::to_string(this->publishing_count_);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
        this->data_publisher_->publish(msg);
    }

private:
    uint32_t                                                publishing_count_;

    rclcpp::Publisher<::std_msgs::msg::string>::SharedPtr   data_publisher_;
    rclcpp::TimerBase::SharedPtr                            timer_;
};
