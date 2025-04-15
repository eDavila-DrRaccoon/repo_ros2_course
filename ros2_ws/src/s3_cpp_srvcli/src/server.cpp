#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class CppService : public rclcpp::Node {
    public:
        CppService()
        : Node("cpp_server_node") {
            RCLCPP_INFO(this->get_logger(), "C++ Server node has been started");
            service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
                "cpp_add_two_ints_service",
                std::bind(&CppService::AddTwoInts_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
            RCLCPP_INFO(this->get_logger(), "Service 'cpp_add_two_ints_service' is ready to receive requests");
        }

    private:
        void AddTwoInts_callback(
            const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
                response->sum = request->a + request->b;
                RCLCPP_INFO(this->get_logger(), "Received request:\na: %ld, b: %ld", request->a, request->b);
                RCLCPP_INFO(this->get_logger(), "Sending back response: sum = %ld", response->sum);
        }

        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<CppService>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
