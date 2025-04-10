#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "s4_custom_interface/msg/hardware_status.hpp"
#include "s4_custom_interface/msg/sphere.hpp"

using namespace std::chrono_literals;

class CppPublisher : public rclcpp::Node{
    public:
        CppPublisher()
        : Node("cpp_publisher_node"){
            RCLCPP_INFO(this->get_logger(), "C++ Publisher node has been started");
            publisher_ = this->create_publisher<s4_custom_interface::msg::HardwareStatus>("cpp_topic", 10);
            publisher2_ = this->create_publisher<s4_custom_interface::msg::Sphere>("cpp_sphere_topic", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&CppPublisher::timer_callback, this));
        }

    private:
        void timer_callback(){
            // HardwareStatus message
            auto message = s4_custom_interface::msg::HardwareStatus();
            message.temperature = 45;
            message.are_motors_up = true;
            message.debug_message = "Eduardo's C++ system runs smoothly";
            message.the_time = this->get_clock()->now();

            RCLCPP_INFO(this->get_logger(), "Publishing Status: Temperature: %ld, Motors Up: %s, Debug Message: %s, Time: %d.%09d",
                        message.temperature,
                        message.are_motors_up ? "True" : "False",
                        message.debug_message.c_str(),
                        message.the_time.sec,
                        message.the_time.nanosec);
            
            publisher_->publish(message);

            // Sphere message
            auto message2 = s4_custom_interface::msg::Sphere();
            message2.center.x = 1.0;
            message2.center.y = 2.0;
            message2.center.z = 3.0;
            message2.radius = 4.0;

            RCLCPP_INFO(this->get_logger(), "Publishing Sphere: x: %f, y: %f, z: %f, Radius: %f",
                        message2.center.x,
                        message2.center.y,
                        message2.center.z,
                        message2.radius);
            
            publisher2_->publish(message2);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<s4_custom_interface::msg::HardwareStatus>::SharedPtr publisher_;
        rclcpp::Publisher<s4_custom_interface::msg::Sphere>::SharedPtr publisher2_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<CppPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}