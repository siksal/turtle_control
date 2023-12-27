#include "rclcpp/rclcpp.hpp"
#include "turtle_control/srv/stop.hpp"

class Server : public rclcpp::Node
{
    public:
        Server() : Node("stop_robot_server")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to stop robot");

            auto stop_robot = [this](
                const std::shared_ptr<turtle_control::srv::Stop::Request> request, 
                std::shared_ptr<turtle_control::srv::Stop::Response> response) -> void{
                    std::string state = "False";
                    response->stop_robot = true;
                    RCLCPP_INFO(rclcpp::get_logger("stop_robot"), "front_wheel: %d m/s", request->front_wheel);
                    RCLCPP_INFO(rclcpp::get_logger("stop_robot"), "rear_wheel: %d m/s", request->rear_wheel);
                    std::cout << "Robot Stopped? " << std::boolalpha << response->stop_robot << std::endl;

            };

            server = this->create_service<turtle_control::srv::Stop>("stop_robot_service", stop_robot);
        }

    private:
        rclcpp::Service<turtle_control::srv::Stop>::SharedPtr server;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Server>());
    rclcpp::shutdown();
    return 0;
}