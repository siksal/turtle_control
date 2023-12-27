#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/kill.hpp"

using namespace std::chrono_literals;

class Test : public rclcpp::Node
{
    public:
        Test() : Node("testing")
        {
            pub = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
            bool is_forward = true;
            bool clockwise = true;

            move_forward(2, 4, is_forward);
            rotate(deg_to_rad(10), deg_to_rad(120), clockwise);
            move_forward(2, 4, is_forward);
            rotate(deg_to_rad(10), deg_to_rad(120), clockwise);
            move_forward(2, 4, is_forward);
            kill_turtle();
        }
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
        rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_srv;

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

        void kill_turtle()
        {
            kill_srv = this->create_client<turtlesim::srv::Kill>("/kill");
            while (!kill_srv->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
            }

            auto request = std::make_shared<turtlesim::srv::Kill::Request>();
            request->name = "turtle1";
            auto result = kill_srv->async_send_request(request);
        }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Test>());
    rclcpp::shutdown();
}