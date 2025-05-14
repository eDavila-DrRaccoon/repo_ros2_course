#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

class CppClientAsync : public rclcpp::Node {
    public:
        CppClientAsync(int64_t a, int64_t b):
            Node("cpp_client_async_node"), a_(a), b_(b) {
                    RCLCPP_INFO(this->get_logger(), "C++ Client node has been started");
                    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("cpp_add_two_ints_service");
                    // Wait for the service to become available
                    while (!client_->wait_for_service(1s)) {
                        if (!rclcpp::ok()) {
                            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                            return;
                        }
                        RCLCPP_INFO(this->get_logger(), "Waiting for service to become available...");
                }
                // Request is called
                send_request();
            }

    private:
        void send_request() {
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = a_;
            request->b = b_;

            /*### Asynchronously send the request ###*/
            // Standard approach (for Foxy)
            auto result = client_->async_send_request(request);

            // Specific approach (for Humble)
            // rclcpp::Client<example_interfaces::srv::AddTwoInts>::FutureAndRequestId future_and_request_id =
            // client_->async_send_request(request);
            // std::shared_future<example_interfaces::srv::AddTwoInts::Response::SharedPtr> result =
            // future_and_request_id.future.share();

            // Spin until the future is complete
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                rclcpp::FutureReturnCode::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "Received response: %ld + %ld = %ld", request.get()->a, request.get()->b, result.get()->sum);
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
            }
        }

        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
        int64_t a_;
        int64_t b_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    if (argc != 3) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: cpp_client_exe a b");
        return 1;
    }
    std::shared_ptr<rclcpp::Node> node = std::make_shared<CppClientAsync>(atoll(argv[1]), atoll(argv[2]));
    node.reset();
    rclcpp::shutdown();
    return 0;
}
