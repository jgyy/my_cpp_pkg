#include "my_cpp_pkg/battery_node.hpp"

using namespace std::chrono_literals;
using SetLed = my_robot_interfaces::srv::SetLed;

BatteryNode::BatteryNode() : Node("battery_node"), battery_state_(true)
{
    client_ = create_client<SetLed>("set_led");
    timer_ = create_wall_timer(
        100ms, std::bind(&BatteryNode::battery_state_callback, this));
    start_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Battery node initialized");
}

void BatteryNode::battery_state_callback()
{
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - start_time_).count() / 1000.0;
    double cycle_time = std::fmod(elapsed, 10.0);
    if (cycle_time >= 4.0 && battery_state_)
    {
        battery_state_ = false;
        RCLCPP_INFO(get_logger(), "Battery empty! Turning on LED...");
        send_led_request(3, true);
    }
    else if (cycle_time < 4.0 && !battery_state_)
    {
        battery_state_ = true;
        RCLCPP_INFO(get_logger(), "Battery full! Turning off LED...");
        send_led_request(3, false);
    }
}

void BatteryNode::send_led_request(int led_number, bool state)
{
    while (!client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service");
            return;
        }
        RCLCPP_INFO(get_logger(), "Service not available, waiting...");
    }
    auto request = std::make_shared<SetLed::Request>();
    request->led_number = led_number;
    request->state = state;
    auto response_received_callback = [this](
                                        rclcpp::Client<SetLed>::SharedFuture future)
    {
        led_response_callback(future);
    };
    client_->async_send_request(request, response_received_callback);
}

void BatteryNode::led_response_callback(rclcpp::Client<SetLed>::SharedFuture future)
{
    try
    {
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_INFO(get_logger(), "Successfully changed LED state");
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Failed to change LED state");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
