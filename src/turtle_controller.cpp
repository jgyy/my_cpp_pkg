#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <memory>
#include <vector>
#include <cmath>
#include <random>

class TurtleController: public rclcpp::Node
{
public:
    TurtleController() : Node("turtle_controller")
    {
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleController::pose_callback, this, std::placeholders::_1));
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);
        kill_client_ = this->create_client<turtlesim::srv::Kill>("kill");
        current_target_id_ = 2;
        RCLCPP_INFO(this->get_logger(), "Turtle Controller initialized");
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        move_to_nearest_turtle();
    }

    void move_to_nearest_turtle()
    {
        if (target_turtles_.empty())
        {
            std::string turtle_name = "turtle_" + std::to_string(current_target_id_);
            target_turtles_.push_back(std::make_pair(
                static_cast<double>(current_target_id_ % 2 == 0 ? 2.0 : 9.0),
                static_cast<double>(current_target_id_ % 4 < 2 ? 2.0 : 9.0)));
        }
        double target_x = target_turtles_[0].first;
        double target_y = target_turtles_[0].second;
        double dx = target_x - current_pose_.x;
        double dy = target_y - current_pose_.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_angle = std::atan2(dy, dx);
        double angle_diff = target_angle - current_pose_.theta;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        auto twist_msg = geometry_msgs::msg::Twist();
        if (distance < 0.5)
        {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            velocity_publisher_->publish(twist_msg);
            auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
            kill_request->name = "turtle_" + std::to_string(current_target_id_);
            kill_client_->async_send_request(kill_request);
            RCLCPP_INFO(this->get_logger(), "Caught turtle_%d!", current_target_id_);
            target_turtles_.clear();
            current_target_id_++;
            return;
        }
        const double ANGULAR_SPEED = 1.5;
        const double LINEAR_SPEED = 2.0;
        const double ANGLE_THRESHOLD = 0.1;
        if (std::abs(angle_diff) > ANGLE_THRESHOLD)
        {
            twist_msg.angular.z = ANGULAR_SPEED * (angle_diff > 0 ? 1.0 : -1.0);
            twist_msg.linear.x = 0.0;
        }
        else
        {
            twist_msg.linear.x = LINEAR_SPEED * (1.0 - std::abs(angle_diff) / M_PI);
            twist_msg.angular.z = angle_diff * 1.0;
        }
        velocity_publisher_->publish(twist_msg);
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::TimerBase::SharedPtr spawn_check_timer_;
    turtlesim::msg::Pose current_pose_;
    std::vector<std::pair<double, double>> target_turtles_;
    int current_target_id_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
