#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0)
    {
        number_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "/number", 10,
            std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));
        counter_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/number_count", 10);

        reset_service_ = this->create_service<example_interfaces::srv::SetBool>(
            "/reset_number_count",
            std::bind(&NumberCounterNode::callbackResetCounter, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Number Counter Node has been started");
    }

private:
    void callbackNumber(const std_msgs::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;

        auto msg_count = std_msgs::msg::Int64();
        msg_count.data = counter_;
        counter_publisher_->publish(msg_count);
    }

    void callbackResetCounter(
        const example_interfaces::srv::SetBool::Request::SharedPtr request,
        example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data) {
            counter_ = 0;
            response->success = true;
            response->message = "Counter has been reset to 0";
        } else {
            response->success = false;
            response->message = "Counter was not reset";
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr number_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr counter_publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr reset_service_;
    int64_t counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
