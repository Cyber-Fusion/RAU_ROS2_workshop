#pragma once

#include <chrono>
#include <functional>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <common_protocol/action/leg_node.hpp>

using namespace std::chrono_literals;
using LegNodeAction = common_protocol::action::LegNode;
using GoalHandlerLegNodeAction = rclcpp_action::ClientGoalHandle<LegNodeAction>;


class HeadNode : public rclcpp::Node {
public:
    HeadNode(const rclcpp::NodeOptions& options)
    : Node("head_node", options)
    , ready_for_new_actions_(true) {
        vision_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "vision_data", 10, std::bind(&HeadNode::on_vision_data, this, std::placeholders::_1));
        leg_node_action_client_ = rclcpp_action::create_client<LegNodeAction>(
            this, "leg_node");
    }

private:
    void on_vision_data(const std_msgs::msg::String& msg);

    void on_leg_goal_response(const GoalHandlerLegNodeAction::SharedPtr& goal_handle);

    void on_leg_feedback(GoalHandlerLegNodeAction::SharedPtr,
        const ::std::shared_ptr<const LegNodeAction::Feedback> feedback);

    void on_leg_result(const GoalHandlerLegNodeAction::WrappedResult& result);

private:
    std::atomic<bool>                                           ready_for_new_actions_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr      vision_subscriber_;
    rclcpp_action::Client<LegNodeAction>::SharedPtr             leg_node_action_client_;    
};
