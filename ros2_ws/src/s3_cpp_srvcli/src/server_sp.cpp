#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <memory>

void AddTwoInts(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
        {
            response->sum = request->a + request->b;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request:\na: %ld", " b: %ld", request->a, request->b);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: sum = %ld", (long int)response->sum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cpp_server_node");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "C++ Server node has been started");
    
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service = node->create_service<example_interfaces::srv::AddTwoInts>("AddTwoInts_cpp", &AddTwoInts);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service 'cpp_add_two_ints_service' is ready to receive requests");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
