#pragma once

#include <thread>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <common_protocol/action/leg_node.hpp>

using LegNodeAction = common_protocol::action::LegNode;
using GoalHandlerLegNodeAction = rclcpp_action::ServerGoalHandle<LegNodeAction>;

class LegNode : public rclcpp::Node {
public:
    LegNode(const rclcpp::NodeOptions& options)
    : Node("leg_node", options) {
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
        std::shared_ptr<const LegNodeAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlerLegNodeAction> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandlerLegNodeAction> goal_handle);
    void execute(const std::shared_ptr<GoalHandlerLegNodeAction> goal_handle);
private:
    rclcpp_action::Server<LegNodeAction>::SharedPtr server_;
};