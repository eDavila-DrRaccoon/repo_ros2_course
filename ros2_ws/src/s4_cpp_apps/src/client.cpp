#include "rclcpp/rclcpp.hpp"
#include "s4_custom_interface/srv/add_three_ints.hpp"

using namespace std::chrono_literals;

class CppClientAsync : public rclcpp::Node{
    public:
        CppClientAsync(int64_t a, int64_t b, int64_t c):
            Node("cpp_client_async_node"), a_(a), b_(b), c_(c) {
                    RCLCPP_INFO(this->get_logger(), "C++ Client node has been started");
                    client_ = this->create_client<s4_custom_interface::srv::AddThreeInts>("cpp_add_three_ints_service");
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
            std::shared_ptr<s4_custom_interface::srv::AddThreeInts::Request> request = std::make_shared<s4_custom_interface::srv::AddThreeInts::Request>();
            request->a = a_;
            request->b = b_;
            request->c = c_;
            // Asynchronously send the request
            rclcpp::Client<s4_custom_interface::srv::AddThreeInts>::FutureAndRequestId future_and_request_id =
            client_->async_send_request(request);
            std::shared_future<s4_custom_interface::srv::AddThreeInts::Response::SharedPtr> result =
            future_and_request_id.future.share();
            // Spin until the future is complete
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Received response: %ld + %ld + %ld = %ld", request.get()->a, request.get()->b, request.get()->c, result.get()->sum);
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Failed to call service add_three_ints");
            }
        }

        rclcpp::Client<s4_custom_interface::srv::AddThreeInts>::SharedPtr client_;
        int64_t a_;
        int64_t b_;
        int64_t c_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    if (argc != 4) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: client_exe a b c");
        return 1;
    }
    std::shared_ptr<rclcpp::Node> node = std::make_shared<CppClientAsync>(atoll(argv[1]), atoll(argv[2]), atoll(argv[3]));
    node.reset();
    rclcpp::shutdown();
    return 0;
}
