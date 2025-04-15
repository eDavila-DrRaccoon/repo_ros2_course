#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "s4_custom_interface/msg/hardware_status.hpp"
#include "s4_custom_interface/msg/sphere.hpp"

using std::placeholders::_1;

class CppSubscriber : public rclcpp::Node{
    public:
        CppSubscriber()
        : Node("cpp_subscriber_node") {
            RCLCPP_INFO(this->get_logger(), "C++ Subscriber node has been started");
            subscription_ = this->create_subscription<s4_custom_interface::msg::HardwareStatus>(
                "py_hs_topic", 10, std::bind(&CppSubscriber::topic_callback, this, _1));
            subscription2_ = this->create_subscription<s4_custom_interface::msg::Sphere>(
                "cpp_sphere_topic", 10, std::bind(&CppSubscriber::sphere_callback, this, _1));
        }

    private:
        void topic_callback(const s4_custom_interface::msg::HardwareStatus::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received HardwareStatus: Temperature: %ld, Motors Up: %s, Debug Message: %s, Time: %d.%09d",
                    msg->temperature,
                    msg->are_motors_up ? "True" : "False",
                    msg->debug_message.c_str(),
                    msg->the_time.sec,
                    msg->the_time.nanosec);
        }

        void sphere_callback(const s4_custom_interface::msg::Sphere::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Sphere: x: %f, y: %f, z: %f, Radius: %f",
                    msg->center.x,
                    msg->center.y,
                    msg->center.z,
                    msg->radius);
        }

        rclcpp::Subscription<s4_custom_interface::msg::HardwareStatus>::SharedPtr subscription_;
        rclcpp::Subscription<s4_custom_interface::msg::Sphere>::SharedPtr subscription2_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<CppSubscriber>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}