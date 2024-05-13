#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using namespace std::chrono_literals;

class MoveInCircle: public rclcpp::Node
{
    public:
        MoveInCircle(): Node("move_in_circle")
        {
            pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            timer = this->create_wall_timer(500ms, std::bind(&MoveInCircle::timer_callback, this));
            RCLCPP_INFO(this->get_logger(), "successfully launched move_in_circle node");
        }

    private:
        void timer_callback()
        {
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = 0.5;
            msg.angular.z = 0.5;
            pub->publish(msg);
        }

    // variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc , char ** argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveInCircle>());
    rclcpp::shutdown();
    return 0;
}