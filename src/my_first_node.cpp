#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("cpp_test"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Node constructor started");
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                counter_++;
                RCLCPP_INFO(this->get_logger(), "Timer callback! Count: %d", counter_);
            });
            
        RCLCPP_INFO(this->get_logger(), "Timer created successfully");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Creating node...");
    auto node = std::make_shared<MyNode>();
    RCLCPP_INFO(node->get_logger(), "Starting spin...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

