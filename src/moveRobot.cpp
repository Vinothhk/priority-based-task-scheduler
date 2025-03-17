// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <nav2_msgs/action/navigate_to_pose.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <string>
// #include <sstream>

// class Nav2Client : public rclcpp::Node {
// public:
//     using NavigateToPose = nav2_msgs::action::NavigateToPose;
//     using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

//     Nav2Client() : Node("nav2_client") {
//         this->declare_parameter("goal_topic", "/priority_goal");

//         // Subscribe to the goal topic (Published by task_scheduler)
//         goal_subscriber_ = this->create_subscription<std_msgs::msg::String>(
//             this->get_parameter("goal_topic").as_string(), 10,
//             std::bind(&Nav2Client::goal_callback, this, std::placeholders::_1));

//         // Create the action client for NavigateToPose
//         action_client_ = rclcpp_action::create_client<NavigateToPose>(
//             this, "navigate_to_pose");

//         RCLCPP_INFO(this->get_logger(), "Nav2 Client Node Initialized.");
//     }

// private:
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_subscriber_;
//     rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

//     void goal_callback(const std_msgs::msg::String::SharedPtr msg) {
//         double x, y;
//         std::istringstream iss(msg->data);
//         iss >> x >> y;

//         if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
//             RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available!");
//             return;
//         }

//         auto goal_msg = NavigateToPose::Goal();
//         goal_msg.pose.header.frame_id = "map"; // Adjust if using a different frame
//         goal_msg.pose.header.stamp = this->now();
//         goal_msg.pose.pose.position.x = x;
//         goal_msg.pose.pose.position.y = y;
//         goal_msg.pose.pose.orientation.w = 1.0; // Default orientation

//         auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//         send_goal_options.result_callback = std::bind(&Nav2Client::goal_result_callback, this, std::placeholders::_1);

//         RCLCPP_INFO(this->get_logger(), "Sending robot to: (%.2f, %.2f)", x, y);
//         action_client_->async_send_goal(goal_msg, send_goal_options);
//     }

//     void goal_result_callback(const GoalHandleNav::WrappedResult& result) {
//         if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
//             RCLCPP_INFO(this->get_logger(), "Navigation completed successfully.");
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Navigation failed.");
//         }
//     }
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<Nav2Client>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

class Nav2Client : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Nav2Client() : Node("nav2_client") {
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        task_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/priority_goal", 10, std::bind(&Nav2Client::goal_callback, this, std::placeholders::_1));
        completion_publisher_ = this->create_publisher<std_msgs::msg::String>("/task_completed", 10);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr completion_publisher_;

    void goal_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::istringstream iss(msg->data);
        double x, y;
        iss >> x >> y;

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map"; // Adjust if using a different frame
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal: (%.1f, %.1f)", x, y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const GoalHandleNavigate::WrappedResult& result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Goal Reached!");

                    // Publish task completion
                    std_msgs::msg::String completion_msg;
                    completion_msg.data = "Task completed";
                    completion_publisher_->publish(completion_msg);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Navigation failed!");
                }
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2Client>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
