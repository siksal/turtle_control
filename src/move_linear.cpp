#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Move : public rclcpp::Node
{
    public:
        Move() : Node("moving")
        {
            pub = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
            double speed;
            double desired_dist;
            bool is_forward;

            std::cout << "Enter speed: ";
            std::cin >> speed;
            std::cout << "Enter desired distance: ";
            std::cin >> desired_dist;
            std::cout << "Enter direction (1-Forward, 0-Backward): ";
            std::cin >> is_forward;

            move_forward(speed, desired_dist, is_forward);
        }
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;

        void move_forward(double speed, double desired_dist, bool is_forward)
        {
            geometry_msgs::msg::Twist speed_msg;
            if(is_forward)
                speed_msg.linear.x = abs(speed);
            else
                speed_msg.linear.x = -abs(speed);
            speed_msg.linear.y = 0;
            speed_msg.linear.z = 0;
            speed_msg.angular.x = 0;
            speed_msg.angular.y = 0;
            speed_msg.angular.z = 0;

            rclcpp::Clock time;
            double t0 = time.now().seconds();
            double current_dist = 0;
            rclcpp::Rate rate(10);

            while(current_dist < desired_dist){
                pub->publish(speed_msg);
                double t1 = time.now().seconds();
                current_dist = abs((speed_msg.linear.x)) * (t1 - t0);
                // std::cout << current_dist << std::endl;
                rate.sleep();
            }

            speed_msg.linear.x = 0;
            pub->publish(speed_msg);
        }    
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Move>());
    rclcpp::shutdown();
}