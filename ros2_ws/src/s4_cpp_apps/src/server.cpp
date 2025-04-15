#include "rclcpp/rclcpp.hpp"
#include "s4_custom_interface/srv/add_three_ints.hpp"

class CppService : public rclcpp::Node{
    public:
        CppService()
        : Node("cpp_server_node") {
            RCLCPP_INFO(this->get_logger(), "C++ Server node has been started");
            service_ = this->create_service<s4_custom_interface::srv::AddThreeInts>(
                "cpp_add_three_ints_service",
                std::bind(&CppService::AddThreeInts_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
            RCLCPP_INFO(this->get_logger(), "Service 'cpp_add_three_ints_service' is ready to receive requests");
        }

    private:
        void AddThreeInts_callback(
            const std::shared_ptr<s4_custom_interface::srv::AddThreeInts::Request> request,
            std::shared_ptr<s4_custom_interface::srv::AddThreeInts::Response> response) {
                response->sum = request->a + request->b + request->c;
                RCLCPP_INFO(this->get_logger(), "Received request:\na: %ld, b: %ld, c: %ld", request->a, request->b, request->c);
                RCLCPP_INFO(this->get_logger(), "Sending back response: sum = %ld", response->sum);
            }

        rclcpp::Service<s4_custom_interface::srv::AddThreeInts>::SharedPtr service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<CppService>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
