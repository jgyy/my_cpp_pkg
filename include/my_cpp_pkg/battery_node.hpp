#ifndef BATTERY_NODE_HPP_
#define BATTERY_NODE_HPP_

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace std::chrono_literals;

class BatteryNode : public rclcpp::Node
{
    public:
        explicit BatteryNode();

    private:
        void battery_state_callback();
        void send_led_request(int led_number, bool state);
        void led_response_callback(
            rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedFuture future);

        rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;
        bool battery_state_;
        std::chrono::steady_clock::time_point start_time_;
};

#endif
