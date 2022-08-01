#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DoJoyPublisher : public rclcpp::Node
{
  public:
    DoJoyPublisher()
    : Node("do_teleop_node")
    {
      publishTwist = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      // publishZ = this->create_publisher<std_msgs::msg::Int32>("robo1_cmd", 10);
      // publishY = this->create_publisher<std_msgs::msg::Int32>("robo2_cmd", 10);
      publishZ = this->create_publisher<std_msgs::msg::String>("robo1_cmd", 10);
      publishY = this->create_publisher<std_msgs::msg::String>("robo2_cmd", 10);

      joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),
        std::bind(&DoJoyPublisher::joyCallback, this, std::placeholders::_1));
    }

  private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
        auto message2 = std_msgs::msg::String();
        auto message3 = std_msgs::msg::String();

        cmd_vel_msg->linear.x = joy_msg->axes[1];
        cmd_vel_msg->linear.y = 0.0;
        cmd_vel_msg->linear.z = 0.0;

        cmd_vel_msg->angular.x = 0.0;
        cmd_vel_msg->angular.y = 0.0;
        cmd_vel_msg->angular.z = joy_msg->axes[0];
        message2.data = "Z: " + std::to_string(joy_msg->axes[4]);
        message3.data = "Y: " + std::to_string(joy_msg->axes[3]);

        publishTwist->publish(std::move(cmd_vel_msg));
        publishZ->publish(message2);
        publishY->publish(message3);

    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publishTwist;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publishZ;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publishY;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DoJoyPublisher>());
  rclcpp::shutdown();
  return 0;
}