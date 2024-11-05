#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::chrono_literals;

class ResetCounterClient : public rclcpp::Node
{
public:
    ResetCounterClient() : Node("reset_counter_client")
    {
        client_ = this->create_client<example_interfaces::srv::SetBool>("/reset_number_count");
        RCLCPP_INFO(this->get_logger(), "Reset Counter Client has been started.");
    }

    void send_request()
    {
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }

        auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
        request->data = true;

        auto result = client_->async_send_request(request,
            [this](rclcpp::Client<example_interfaces::srv::SetBool>::SharedFuture future) {
                auto result = future.get();
                RCLCPP_INFO(this->get_logger(), "Result: %s", result->message.c_str());
            });
    }

private:
    rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResetCounterClient>();
    node->send_request();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
