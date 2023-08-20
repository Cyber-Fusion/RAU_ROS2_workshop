#pragma once

#include <thread>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <common_protocol/action/leg_node.hpp>

using LegNodeAction = common_protocol::action::LegNode;
using GoalHandlerLegNodeAction = rclcpp_action::ServerGoalHandle<LegNodeAction>;
using namespace std::chrono_literals;

class LegNode : public rclcpp::Node {
public:

    LegNode()
    : Node("leg_node") {
        server_ = rclcpp_action::create_server<LegNodeAction>(
            this,
            "leg_node",
            std::bind(&LegNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&LegNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&LegNode::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const LegNodeAction::Goal> goal) {
        
        RCLCPP_INFO(this->get_logger(), "Received goal request with order '%d'.", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlerLegNodeAction> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlerLegNodeAction> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing in separate thread.");
        ::std::thread{std::bind(&LegNode::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandlerLegNodeAction> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal.");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<LegNodeAction::Feedback>();
        auto& sequence = feedback->partial_sequence;
        auto result = std::make_shared<LegNodeAction::Result>();

        for (int i = 0; i < 5; ++i) {
            std::this_thread::sleep_for(100ms);
            if (goal_handle->is_canceling()) {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            sequence.push_back(i);

            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");
        }
        
        // Check if goal is done.
        if (rclcpp::ok()) {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

private:
    rclcpp_action::Server<LegNodeAction>::SharedPtr server_;
};