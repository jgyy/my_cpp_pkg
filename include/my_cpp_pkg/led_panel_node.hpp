#ifndef LED_PANEL_NODE_HPP_
#define LED_PANEL_NODE_HPP_

#include <memory>
#include <array>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"

class LedPanelNode : public rclcpp::Node
{
    public:
        explicit LedPanelNode();

    private:
        void set_led_callback(
            const std::shared_ptr<my_robot_interfaces::srv::SetLed::Request> request,
            std::shared_ptr<my_robot_interfaces::srv::SetLed::Response> response);
        void publish_led_states();
        void checkLedStates();

        rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr service_;
        rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::array<bool, 3> led_states_;
};

#endif
