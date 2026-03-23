#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class CppSubscriber : public rclcpp::Node
{
    public:
        CppSubscriber():
        Node("cpp_subscriber_node") {
            RCLCPP_INFO(this->get_logger(), "C++ Subscriber node has been started");
            subscription_ = this->create_subscription<std_msgs::msg::String>(
            "py_str_topic", 10, std::bind(&CppSubscriber::topic_callback, this, _1));
        }

    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<CppSubscriber>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}