#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberCounter : public rclcpp::Node
{
public:
    NumberCounter() : Node("number_counter"), counter_(0)
    {
        subscription_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10,
            std::bind(&NumberCounter::number_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

        RCLCPP_INFO(this->get_logger(), "Number Counter has been started");
    }

private:
    void number_callback(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;

        auto count_msg = example_interfaces::msg::Int64();
        count_msg.data = counter_;
        publisher_->publish(count_msg);

        RCLCPP_INFO(this->get_logger(), "Counter updated: %ld", counter_);
    }

    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscription_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    int64_t counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
