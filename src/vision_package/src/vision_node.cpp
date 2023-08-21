#include <vision_package/vision_node.hpp>


void VisionNode::on_timer_ringing() {
    RCLCPP_INFO(this->get_logger(), "Publishing data");
    auto msg = std_msgs::msg::String();
    msg.data = "Current status of vision: " + std::to_string(this->published_msg_count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    this->data_publisher_->publish(msg);
}
