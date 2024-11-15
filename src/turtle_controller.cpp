#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <memory>
#include <vector>
#include <cmath>
#include <random>
#include <map>
#include <string>

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
        turtle_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TurtleController::check_for_new_turtles, this));
        current_target_id_ = 2;
        RCLCPP_INFO(this->get_logger(), "Turtle Controller initialized");
    }

private:
    void check_for_new_turtles()
    {
        std::string turtle_name = "turtle_" + std::to_string(current_target_id_);
        if (turtle_pose_subscribers_.find(turtle_name)
            == turtle_pose_subscribers_.end())
        {
            try
            {
                auto sub = this->create_subscription<turtlesim::msg::Pose>(
                    "/" + turtle_name + "/pose", 10,
                    [this, turtle_name](const turtlesim::msg::Pose::SharedPtr msg)
                    {
                        turtle_poses_[turtle_name] = *msg;
                    });
                turtle_pose_subscribers_[turtle_name] = sub;
                RCLCPP_INFO(this->get_logger(), "Subscribed to %s",
                    turtle_name.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to %s: %s",
                    turtle_name.c_str(), e.what());
            }
        }
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        move_to_turtle();
    }

    void move_to_turtle()
    {
        std::string target_name = "turtle_" + std::to_string(current_target_id_);
        if (turtle_poses_.find(target_name) == turtle_poses_.end())
        {
            return;
        }
        auto &target_pose = turtle_poses_[target_name];
        double dx = target_pose.x - current_pose_.x;
        double dy = target_pose.y - current_pose_.y;
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
            kill_request->name = target_name;
            kill_client_->async_send_request(kill_request);
            RCLCPP_INFO(this->get_logger(), "Caught %s!", target_name.c_str());
            turtle_poses_.erase(target_name);
            turtle_pose_subscribers_.erase(target_name);
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
    rclcpp::TimerBase::SharedPtr turtle_check_timer_;
    std::map<std::string, rclcpp::Subscription<
        turtlesim::msg::Pose>::SharedPtr> turtle_pose_subscribers_;
    std::map<std::string, turtlesim::msg::Pose> turtle_poses_;
    turtlesim::msg::Pose current_pose_;
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
