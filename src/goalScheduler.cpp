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
// bool has_pending_task = false;
// Task pending_task;  // Store interrupted task

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
//     task_queue.push(new_task);
//     RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "New task received: (%.1f, %.1f) with priority %d", x, y, priority);

//     if (!is_executing || new_task.priority > current_task.priority) {
//         // If a higher priority task comes in, store the current task for later
//         if (is_executing && new_task.priority > current_task.priority) {
//             has_pending_task = true;
//             pending_task = current_task;
//         }

//         is_executing = true;
//         current_task = new_task;
//         std_msgs::msg::String task_msg;
//         task_msg.data = std::to_string(current_task.x) + " " + std::to_string(current_task.y);
//         task_publisher->publish(task_msg);
//         RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "Sent new goal: (%.1f, %.1f)", current_task.x, current_task.y);
//     }
// }

// void resume_previous_task() {
//     std::lock_guard<std::mutex> lock(queue_mutex);
//     if (has_pending_task) {
//         RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "Resuming previous task: (%.1f, %.1f)", pending_task.x, pending_task.y);
//         current_task = pending_task;
//         std_msgs::msg::String task_msg;
//         task_msg.data = std::to_string(current_task.x) + " " + std::to_string(current_task.y);
//         task_publisher->publish(task_msg);
//         has_pending_task = false;
//     } else {
//         RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "No pending tasks. Robot is idle.");
//         is_executing = false;
//     }
// }

// void completion_callback(const std_msgs::msg::String::SharedPtr msg) {
//     RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "Task completed: %s", msg->data.c_str());
//     resume_previous_task();  // Try to resume previous task after finishing
// }

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("task_scheduler");

//     task_publisher = node->create_publisher<std_msgs::msg::String>("/priority_goal", 10);
//     auto task_subscriber = node->create_subscription<std_msgs::msg::String>(
//         "/new_delivery_request", 10, task_callback);

//     auto completion_subscriber = node->create_subscription<std_msgs::msg::String>(
//         "/task_completed", 10, completion_callback);  // Subscribing to task completion signal

//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

// this works fine

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
//     task_queue.push(new_task);
//     RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "New task received: (%.1f, %.1f) with priority %d", x, y, priority);

//     // If not executing or received a higher priority task, switch immediately
//     if (!is_executing || new_task.priority > current_task.priority) {
//         is_executing = true;
//         current_task = new_task;
//         std_msgs::msg::String task_msg;
//         task_msg.data = std::to_string(current_task.x) + " " + std::to_string(current_task.y);
//         task_publisher->publish(task_msg);
//         RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "Sent new goal: (%.1f, %.1f)", current_task.x, current_task.y);
//     }
// }

// void process_next_task() {
//     std::lock_guard<std::mutex> lock(queue_mutex);

//     if (!task_queue.empty()) {
//         current_task = task_queue.top();
//         task_queue.pop();

//         RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "Starting next task: (%.1f, %.1f) with priority %d", 
//                     current_task.x, current_task.y, current_task.priority);
        
//         std_msgs::msg::String task_msg;
//         task_msg.data = std::to_string(current_task.x) + " " + std::to_string(current_task.y);
//         task_publisher->publish(task_msg);
//         is_executing = true;
//     } else {
//         RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "No tasks left. Robot is idle.");
//         is_executing = false;
//     }
// }

// void completion_callback(const std_msgs::msg::String::SharedPtr msg) {
//     RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "Task completed: %s", msg->data.c_str());
//     process_next_task();  // Instead of resuming, check the full queue
// }

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("task_scheduler");

//     task_publisher = node->create_publisher<std_msgs::msg::String>("/priority_goal", 10);
//     auto task_subscriber = node->create_subscription<std_msgs::msg::String>(
//         "/new_delivery_request", 10, task_callback);

//     auto completion_subscriber = node->create_subscription<std_msgs::msg::String>(
//         "/task_completed", 10, completion_callback);  // Subscribing to task completion signal

//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <queue>
#include <mutex>
#include <sstream>
#include <cmath>

struct Task {
    double x, y;
    int priority;
    bool operator<(const Task& other) const {
        return priority < other.priority; // Higher priority first
    }
};

std::priority_queue<Task> task_queue;
std::mutex queue_mutex;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_publisher;
Task current_task;
bool is_executing = false;

int calculate_priority(int urgency, int distance, int order_value) {
    return (3 * urgency) + (1 * order_value) - (2 * distance);
}

// Function to send next task
void send_next_task() {
    std::lock_guard<std::mutex> lock(queue_mutex);
    
    if (!task_queue.empty()) {
        current_task = task_queue.top();
        task_queue.pop();
        is_executing = true;

        std_msgs::msg::String task_msg;
        task_msg.data = std::to_string(current_task.x) + " " + std::to_string(current_task.y);
        task_publisher->publish(task_msg);

        RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "Sent new goal: (%.1f, %.1f) with priority %d", 
                    current_task.x, current_task.y, current_task.priority);
    } else {
        is_executing = false;
        RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "No more tasks. Robot is idle.");
    }
}

// Callback for receiving new tasks
void task_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::istringstream iss(msg->data);
    double x, y;
    int urgency, order_value;

    // Ensure valid input format
    if (!(iss >> x >> y >> urgency >> order_value)) {
        RCLCPP_WARN(rclcpp::get_logger("task_scheduler"), "Invalid task format received: %s", msg->data.c_str());
        return;
    }

    // Calculate priority correctly
    int distance = static_cast<int>(sqrt(x * x + y * y));  // Euclidean distance
    int priority = calculate_priority(urgency, distance, order_value);

    Task new_task{x, y, priority};

    bool should_send = false;

    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        task_queue.push(new_task);
        RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "New task received: (%.1f, %.1f) with priority %d", x, y, priority);

        // If no task is executing OR the new task has higher priority, interrupt current task
        if (!is_executing || new_task.priority > current_task.priority) {
            if (is_executing) {
                task_queue.push(current_task);  // Reinsert interrupted task
                RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "Paused task: (%.1f, %.1f) for higher priority", 
                            current_task.x, current_task.y);
            }
            should_send = true;
        }
    }

    if (should_send) {
        send_next_task();
    }
}

// Callback for task completion
void completion_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "Task completed: %s", msg->data.c_str());
    send_next_task();  // Pick the next task in the queue
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("task_scheduler");

    task_publisher = node->create_publisher<std_msgs::msg::String>("/priority_goal", 10);
    auto task_subscriber = node->create_subscription<std_msgs::msg::String>(
        "/new_delivery_request", 10, task_callback);

    auto completion_subscriber = node->create_subscription<std_msgs::msg::String>(
        "/task_completed", 10, completion_callback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
