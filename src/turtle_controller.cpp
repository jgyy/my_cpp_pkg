#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/kill.hpp>
#include <memory>
#include <vector>
#include <cmath>

class TurtleController: public rclcpp::Node
{
public:
    TurtleController() : Node("turtle_controller")
    {
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleController::pose_callback, this, std::placeholders::_1));
        velocity_publisher_ = this->create_publisher<geometry_msg::msg::Twist>(
            "/turtle1/cmd_vel", 10);
        kill_client_ = this->create_client<turtlesim::srv::Kill>("kill");
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
        if (target_turtles._empty())
        {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            velocity_publisher_->publish(twist_msg);
            return;
        }
        double min_distance = std::numeric_limits<double>::max();
        size_t nearest_index = 0;
        for (size_t i = 0; i < target_turtles_.size(); ++i)
        {
            double dx = target_turtles_[i].first - current_pose_.x;
            double dy = target_turtles_[i].second - current_pose_.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_index = i;
            }
        }
        double target_x = target_turtles_[nearest_index].first;
        double target_y = target_turtles_[nearest_index].second;
        double dx = target_x - current_pose_.x;
        double dy = target_y - current_pose_.y;
        double target_angle = std::atan2(dy, dx);
        double angle_diff = target_angle - current_pose_.theta;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        auto twist_msg = getmetry_msgs::msg::Twist();
        if (min_distance < 0.5)
        {
            target_turtles_.erase(target_turtles_.begin() + nearest_index);
            auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
            kill_request->name = "turtle_" + std::to_string(nearest_index + 2);
            kill_client_->async_send_request(kill_request);
            RCLCPP_INFO(this->get_logger(), "Caught turtle_%d!", nearest_index + 2);
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
            twist_msg.linear.x = LINEAR_SPEED;
            twist_msg.angular.z = angle_diff * 0.5;
        }
        velocity_publisher_->publish(twist_msg);
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    turtlesim::msg::Pose current_pose_;
    std::vector<std::pair<double, double>> target_turtles_;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
