#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/kill.hpp>
#include <memory>
#include <random>

class TurtleSpawner: public rclcpp::Node
{
public:
    TurtleSpawner() : Node("turtle_spawner")
    {
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        spawn_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&TurtleSpawner::spawn_turtle, this));
        RCLCPP_INFO(this->get_logger(), "Turtle Spawner initialized");
    }

private:
    void spawn_turtle()
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<> dis(0.0, 11.0);
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = dis(gen);
        request->y = dis(gen);
        request->theta = dis(gen) * 2 * M_PI;
        request->name = "turtle_" + std::to_string(turtle_count_++);
        spawn_client_->async_send_request(request);
    }

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    int turtle_count_ = 2;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
