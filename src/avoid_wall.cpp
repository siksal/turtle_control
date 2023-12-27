#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class AvoidWall : public rclcpp::Node
{
  public:
    AvoidWall() : Node("avoid_wall")
    {
      speed_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "turtle1/cmd_vel", 10);

      pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10, std::bind(&AvoidWall::pose_callback, this, _1));

      timer_ = this->create_wall_timer(
        10ms, std::bind(&AvoidWall::speed_timer_callback, this));
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist speed;
    float pose_x;
    float pose_y;

    void pose_callback(const turtlesim::msg::Pose::SharedPtr turtle_pose)
    {
      pose_x = turtle_pose->x;
      pose_y = turtle_pose->y;
    }

    void speed_timer_callback()
    {
      speed.linear.x = 2.0;
      if(pose_x <= 1.0 || pose_x >= 10.0 || pose_y <= 1.0 || pose_y >= 10.0){
        RCLCPP_INFO(this->get_logger(), "Close to the wall, now turning");
        speed.angular.z = 22/7; // Turn 180 degress counterclockwise
      }
      else{
        speed.angular.z = 0.0;
      }
      speed_pub_->publish(speed);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidWall>());
  rclcpp::shutdown();
  return 0;
}