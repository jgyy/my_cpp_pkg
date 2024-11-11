#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublisher : public rclcpp::Node
{
public:
    NumberPublisher() : Node("number_publisher"), number_(2)
    {
        this->declare_parameter("name");

        publisher_ = this->create_publisher<example_interfaces::msg::Int64>(
                                                                        "number", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NumberPublisher::publish_number, this));

        RCLCPP_INFO(this->get_logger(), "Number Publisher has been started");
    }

private:
    void publish_number()
    {
        auto message = example_interfaces::msg::Int64();
        message.data = number_;
        publisher_->publish(message);
    }

    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    const int64_t number_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
