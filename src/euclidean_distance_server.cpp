#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

float x_1;
float x_2;
float y_1;
float y_2;

void pose_callback(const turtlesim::msg::Pose::SharedPtr turtle_pose)
{
    x_1 = turtle_pose->x;
    y_1 = turtle_pose->y;
}

void get_distance(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
    x_2 = request->a;
    y_2 = request->b;

    response->sum = sqrt(pow((x_2 - x_1), 2) + pow((y_2 - y_1), 2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming coordinates)\n(%.2f, %.2f) (%.2f, %.2f)",
                x_1, y_1, x_2, y_2);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%ld]", response->sum);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("euclidean_distance_server");

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;

    pose_sub_ = node->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, pose_callback);
    server_ = node->create_service<example_interfaces::srv::AddTwoInts>("dist_two_points", &get_distance);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to get distance between two points.");
    rclcpp::spin(node);
    rclcpp::shutdown();
}