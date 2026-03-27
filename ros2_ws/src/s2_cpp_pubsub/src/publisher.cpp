#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class CppPublisher : public rclcpp::Node {
    public:
        CppPublisher():
        Node("cpp_publisher_node"), count_(0) {
            RCLCPP_INFO(this->get_logger(), "C++ Publisher node has been started");
            publisher_ = this->create_publisher<std_msgs::msg::String>("cpp_str_topic", 10);
            timer_ = this->create_wall_timer(
            500ms, std::bind(&CppPublisher::timer_callback, this));
        }

    private:
        void timer_callback() {
            std_msgs::msg::String message = std_msgs::msg::String();
            message.data = "Hello, this is Eduardo from the C++ Publisher: " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<CppPublisher>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
