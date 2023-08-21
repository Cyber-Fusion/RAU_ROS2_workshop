#include <head_package/head_node.hpp>


#include <sstream>

std::string id_to_string(const rclcpp_action::GoalUUID& id) {
    std::stringstream ss;
    ss << std::hex << std::noskipws;
    for (size_t i = 0; i < id.size(); i++) {
        if (i == 5 || i == 9) {
            ss << "-";
        }

        ss << std::setw(2) << std::setfill('0') << (int)id[i];
    }

    return ss.str();
}

void HeadNode::on_vision_data(const std_msgs::msg::String& msg) {
    RCLCPP_INFO(this->get_logger(), "Data from vision node: '%s'", msg.data.c_str());

    bool expecting_ready = true;
    if (!this->ready_for_new_actions_.compare_exchange_weak(expecting_ready, false)) {
        RCLCPP_INFO(this->get_logger(), "Previous action is in progress. We can't process current.");
        return;
    }

    if (!this->leg_node_action_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available.");
        rclcpp::shutdown();
    }

    auto goal_msg = LegNodeAction::Goal();
    goal_msg.order = 10;
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<LegNodeAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&HeadNode::on_leg_goal_response, this, ::std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&HeadNode::on_leg_feedback, this, ::std::placeholders::_1, ::std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&HeadNode::on_leg_result, this, ::std::placeholders::_1);

    this->leg_node_action_client_->async_send_goal(goal_msg, send_goal_options);
}


void HeadNode::on_leg_goal_response(const ::GoalHandlerLegNodeAction::SharedPtr& goal_handle) {
    std::string goal_id = id_to_string(goal_handle->get_goal_id());
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(),
            "The goal was rejected by the server. Goal ID: '%s'.", goal_id.c_str());
        this->ready_for_new_actions_.store(true);
    } else {
        RCLCPP_INFO(this->get_logger(),
            "The goal was accepted by the server. Goal ID: '%s'.", goal_id.c_str());
        this->ready_for_new_actions_.store(false);
    }
}

void HeadNode::on_leg_feedback(GoalHandlerLegNodeAction::SharedPtr goal_handle,
    const std::shared_ptr<const LegNodeAction::Feedback> feedback) {
    
    std::string goal_id = id_to_string(goal_handle->get_goal_id());
    std::stringstream ss;
    ss << "Feeedback sequence: ";
    for (auto num : feedback->partial_sequence) {
        ss << num << " ";
    }
    ss << "Goal ID: " << goal_id << ".";

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

void HeadNode::on_leg_result(const GoalHandlerLegNodeAction::WrappedResult& result) {
    std::string goal_id = id_to_string(result.goal_id);
    bool broken = false;

    switch(result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),
                "Goal was succeeded. Goal ID: '%s'.", id_to_string(result.goal_id).c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(),
                "Goal was aborted. Goal ID: '%s'.", id_to_string(result.goal_id).c_str());
            broken = true;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled. Goal ID: '%s'.",
                id_to_string(result.goal_id).c_str());
            broken = true;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code. Goal ID: '%s'.",
                id_to_string(result.goal_id).c_str());
            broken = true;
            return;
    }

    if (!broken) {
        std::stringstream ss;
        ss << "Result: ";
        for (auto num : result.result->sequence) {
            ss << num << " ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    RCLCPP_INFO(this->get_logger(), "################################################################");
    RCLCPP_INFO(this->get_logger(), "################################################################");
    RCLCPP_INFO(this->get_logger(), "################################################################");
    std::this_thread::sleep_for(5s);

    this->ready_for_new_actions_.store(true);
}




