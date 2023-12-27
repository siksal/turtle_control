#define _USE_MATH_FUNCTIONS
#include <random>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

unsigned int count = 0;
double pose_x;
double pose_y;
double pose_theta;
const std::string turtle_prefix = "turtle_";
const double tolerance = 0.6;

bool spawn_first = true;
bool stop = false;

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<> dist1(1, 10);
std::uniform_int_distribution<> dist2(0, 2*M_PI);

class TurtleCatcher : public rclcpp::Node
{
    public:
        TurtleCatcher() : Node("turtle_catcher")
        {
            pose_sub = this->create_subscription<turtlesim::msg::Pose>(
                "/turtle1/pose", 10, std::bind(&TurtleCatcher::pose_cb, this, _1));

            vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
                "/turtle1/cmd_vel", 10);

            timer = this->create_wall_timer(
                0.01s, std::bind(&TurtleCatcher::go_to_goal, this));
        }
    private:
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_srv;
        rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_srv;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
        rclcpp::TimerBase::SharedPtr timer;
        geometry_msgs::msg::Twist vel_msg;
        turtlesim::msg::Pose gpose;

        void spawn_turtle(turtlesim::msg::Pose goal_pose)
        {
            spawn_srv = this->create_client<turtlesim::srv::Spawn>("/spawn");
            while (!spawn_srv->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
            }

            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            request->x = goal_pose.x;
            request->y = goal_pose.y;
            request->theta = goal_pose.theta;
            request->name = turtle_prefix + std::to_string(count);

            auto result = spawn_srv->async_send_request(request);
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
            request->name = turtle_prefix + std::to_string(count);
            auto result = kill_srv->async_send_request(request);
        }

        void pose_cb(turtlesim::msg::Pose::SharedPtr pose)
        {
            pose_x = pose->x;
            pose_y = pose->y;
            pose_theta = pose->theta;
        }

        double euclidean_dist(turtlesim::msg::Pose goal_pose)
        {
            return sqrt(pow((goal_pose.x - pose_x), 2) + pow((goal_pose.y - pose_y), 2));
        }

        double linear_vel(turtlesim::msg::Pose goal_pose, double constant=1.5)
        {
            return constant * euclidean_dist(goal_pose);
        }

        double steering_angle(turtlesim::msg::Pose goal_pose)
        {
            return atan2(goal_pose.y - pose_y, goal_pose.x - pose_x);
        }

        double angular_vel(turtlesim::msg::Pose goal_pose, double constant=6)
        {
            return constant * (steering_angle(goal_pose) - pose_theta);
        }

        void go_to_goal()
        {
            if(spawn_first == true){
                count = 1;
                gpose.x = dist1(gen);
                gpose.y = dist1(gen);
                gpose.theta = dist2(gen);
                spawn_turtle(gpose);
                spawn_first = false;
            }

            if(euclidean_dist(gpose) >= tolerance && spawn_first == false){
                vel_msg.linear.x = linear_vel(gpose);
                vel_msg.linear.y = 0.0;
                vel_msg.linear.z = 0.0;

                vel_msg.angular.x = 0.0;
                vel_msg.angular.y = 0.0;
                vel_msg.angular.z = angular_vel(gpose);

                vel_pub->publish(vel_msg);
            }

            if(euclidean_dist(gpose) < tolerance && spawn_first == false){
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
                vel_pub->publish(vel_msg);
                stop = true;
            }

            if(stop == true){
                kill_turtle();
                gpose.x = dist1(gen);
                gpose.y = dist1(gen);
                gpose.theta = dist2(gen);
                count++;
                spawn_turtle(gpose);
                stop = false;
            }
        }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleCatcher>());
    rclcpp::shutdown();
}

