#pragma once

#include <chrono>
#include <functional>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <common_protocol/action/leg_node.hpp>

using namespace std::chrono_literals;
using LegNodeAction = common_protocol::action::LegNode;
using GoalHandlerLegNodeAction = rclcpp_action::ClientGoalHandle<LegNodeAction>;


class HeadNode : public rclcpp::Node {
public:
    HeadNode()
    : Node("head_node")
    , ready_for_new_actions_(true) {
        vision_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "vision_data", 10, std::bind(&HeadNode::on_vision_data, this, std::placeholders::_1));
        leg_node_action_client_ = rclcpp_action::create_client<LegNodeAction>(
            this, "leg_node");
    }

private:
    void on_vision_data(const std_msgs::msg::String& msg) {
        RCLCPP_INFO(this->get_logger(), "Data from vision node: '%s'", msg.data.c_str());

        if (!this->ready_for_new_actions_) {
            RCLCPP_INFO(this->get_logger(), "Previous action is in progress. We can't process current.");
            return;
        }
        // TODO:: Emulate hard computing process by sleeping.

        if (!this->leg_node_action_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available.");
            rclcpp::shutdown();
        }

        auto goal_msg = LegNodeAction::Goal();
        goal_msg.order = 10;
        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<LegNodeAction>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            ::std::bind(&HeadNode::on_vision_goal_response, this, ::std::placeholders::_1);
        send_goal_options.feedback_callback =
            ::std::bind(&HeadNode::on_vision_feedback, this, ::std::placeholders::_1, ::std::placeholders::_2);
        send_goal_options.result_callback =
            ::std::bind(&HeadNode::on_vision_result, this, ::std::placeholders::_1);

        this->leg_node_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void on_vision_goal_response(const ::GoalHandlerLegNodeAction::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "The goal was rejected by the server.");
        } else {
            RCLCPP_INFO(this->get_logger(), "The goal was accepted by the server.");
            this->ready_for_new_actions_ = false;
        }
    }

    void on_vision_feedback(::GoalHandlerLegNodeAction::SharedPtr,
        const ::std::shared_ptr<const LegNodeAction::Feedback> feedback) {

        std::stringstream ss;
        ss << "Feeedback sequence: ";
        for (auto num : feedback->partial_sequence) {
            ss << num << " ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void on_vision_result(const ::GoalHandlerLegNodeAction::WrappedResult& result) {
        this->ready_for_new_actions_ = true;

        switch(result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was succeeded.");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted.");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled.");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                return;
        }

        std::stringstream ss;
        ss << "Result: ";
        for (auto num : result.result->sequence) {
            ss << num << " ";
        }

        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

private:
    // Use std::atomic<bool> for avoiding race concitions.
    bool                                                        ready_for_new_actions_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr      vision_subscriber_;
    rclcpp_action::Client<LegNodeAction>::SharedPtr             leg_node_action_client_;    
};
