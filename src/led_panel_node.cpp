#include "my_cpp_pkg/led_panel_node.hpp"

using namespace std::chrono_literals;
using SetLed = my_robot_interfaces::srv::SetLed;
using LedStates = my_robot_interfaces::msg::LedStates;

LedPanelNode::LedPanelNode() : Node("led_panel_node")
{
    led_states_.fill(false);
    service_ = create_service<SetLed>("set_led",
        std::bind(&LedPanelNode::set_led_callback, this,
            std::placeholders::_1, std::placeholders::_2));
    publisher_ = create_publisher<LedStates>("led_panel_state", 10);
    timer_ = create_wall_timer(
        100ms, std::bind(&LedPanelNode::publish_led_states, this));
    RCLCPP_INFO(get_logger(), "LED panel node initialized");
}

void LedPanelNode::set_led_callback(
    const std::shared_ptr<SetLed::Request> request,
    std::shared_ptr<SetLed::Response> response)
{
    if (request->led_number < 1 || request->led_number > 3)
    {
        RCLCPP_ERROR(get_logger(), "Invalid LED number: %d", request->led_number);
        response->success = false;
        return;
    }
    size_t led_index = static_cast<size_t>(request->led_number - 1);
    led_states_[led_index] = request->state;
    RCLCPP_INFO(get_logger(), "LED %d turned %s",
        request->led_number, request->state ? "on" : "off");
    response->success = true;
}

void LedPanelNode::publish_led_states()
{
    auto message = std::make_unique<LedStates>();
    for (size_t i = 0; i < led_states_.size(); ++i)
    {
        message->states[i] = led_states_[i];
    }
    publisher_->publish(std::move(message));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
