// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <queue>
// #include <mutex>
// #include <vector>
// #include <sstream>

// struct Task {
//     double x, y;
//     int priority;
//     bool operator<(const Task& other) const {
//         return priority < other.priority; // Higher priority first
//     }
// };

// std::priority_queue<Task> task_queue;
// std::mutex queue_mutex;
// rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_publisher;
// Task current_task;
// bool is_executing = false;

// int calculate_priority(int urgency, int distance, int order_value) {
//     return (3 * urgency) + (1 * order_value) - (2 * distance);
// }

// void task_callback(const std_msgs::msg::String::SharedPtr msg) {
//     std::istringstream iss(msg->data);
//     double x, y;
//     int urgency, order_value;
//     iss >> x >> y >> urgency >> order_value;

//     int priority = calculate_priority(urgency, static_cast<int>(x + y), order_value);
//     Task new_task{x, y, priority};

//     std::lock_guard<std::mutex> lock(queue_mutex);

//     if (is_executing) {
//         // If a higher-priority task arrives, store the current task back in the queue
//         if (new_task.priority > current_task.priority) {
//             RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), 
//                         "Higher priority task received! Pausing current task and switching.");

//             task_queue.push(current_task); // Save current task for resumption
//             current_task = new_task;       // Switch to new task
//         } else {
//             task_queue.push(new_task); // Add new task to queue
//             return;
//         }
//     } else {
//         current_task = new_task;
//         is_executing = true;
//     }

//     // Publish the new task to be executed
//     std_msgs::msg::String task_msg;
//     task_msg.data = std::to_string(current_task.x) + " " + std::to_string(current_task.y);
//     task_publisher->publish(task_msg);
//     RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "Sent new goal: (%.1f, %.1f)", current_task.x, current_task.y);
// }

// void task_completed_callback() {
//     std::lock_guard<std::mutex> lock(queue_mutex);
//     is_executing = false;

//     if (!task_queue.empty()) {
//         current_task = task_queue.top();
//         task_queue.pop();
//         is_executing = true;

//         std_msgs::msg::String task_msg;
//         task_msg.data = std::to_string(current_task.x) + " " + std::to_string(current_task.y);
//         task_publisher->publish(task_msg);
//         RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "Resumed previous task: (%.1f, %.1f)", 
//                     current_task.x, current_task.y);
//     } else {
//         RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "No pending tasks.");
//     }
// }

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("task_scheduler");

//     task_publisher = node->create_publisher<std_msgs::msg::String>("/priority_goal", 10);
//     auto task_subscriber = node->create_subscription<std_msgs::msg::String>(
//         "/new_delivery_request", 10, task_callback);

//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <sstream>

class ManualTaskPublisher : public rclcpp::Node {
public:
    ManualTaskPublisher() : Node("manual_task_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/new_delivery_request", 10);
        input_thread_ = std::thread(&ManualTaskPublisher::input_loop, this);
    }

    ~ManualTaskPublisher() {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void input_loop() {
        while (rclcpp::ok()) {
            double x, y;
            int urgency, order_value;
            std::cout << "Enter goal x, y, urgency, and order value (separated by spaces): ";
            std::cin >> x >> y >> urgency >> order_value;
            
            std_msgs::msg::String msg;
            std::stringstream ss;
            ss << x << " " << y << " " << urgency << " " << order_value;
            msg.data = ss.str();
            
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published task: %s", msg.data.c_str());
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::thread input_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManualTaskPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
