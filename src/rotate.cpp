#define _USE_MATH_DEFINES
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class Rotate : public rclcpp::Node
{
    public:
        Rotate() : Node("rotating")
        {
            pub = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

            double ang_speed;
            double desired_angle;
            bool clockwise;

            std::cout << "Enter angular speed: ";
            std::cin >> ang_speed;
            std::cout << "Enter desired angle: ";
            std::cin >> desired_angle;
            std::cout << "Enter direction (1-Counterclockwise, 0-Clockwise): ";
            std::cin >> clockwise;

            rotate(deg_to_rad(ang_speed), deg_to_rad(desired_angle), clockwise);
        }
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;

        void rotate(double ang_speed, double desired_angle, bool clockwise)
        {
            geometry_msgs::msg::Twist ang_speed_msg;
            if(clockwise)
                ang_speed_msg.angular.z = abs(ang_speed);
            else
                ang_speed_msg.angular.z = -abs(ang_speed);
            ang_speed_msg.angular.x = 0;
            ang_speed_msg.angular.y = 0;
            ang_speed_msg.linear.x = 0;
            ang_speed_msg.linear.y = 0;
            ang_speed_msg.linear.z = 0;

            rclcpp::Clock time;
            double t0 = time.now().seconds();
            double current_angle = 0;
            rclcpp::Rate rate(10);

            while(current_angle < desired_angle){
                pub->publish(ang_speed_msg);
                double t1 = time.now().seconds();
                current_angle = abs((ang_speed_msg.angular.z)) * (t1 - t0);
                // std::cout << current_angle << std::endl;
                rate.sleep();
            }

            ang_speed_msg.angular.z = 0;
            pub->publish(ang_speed_msg);
        }

        double deg_to_rad(double ang_in_deg)
        {
            return ang_in_deg * M_PI / 180;
        }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Rotate>());
    rclcpp::shutdown();
}