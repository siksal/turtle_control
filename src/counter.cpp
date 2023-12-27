#include "rclcpp/rclcpp.hpp"
#include "turtle_control/msg/message_counter.hpp"

using namespace std::chrono_literals;

class Counter : public rclcpp::Node
{
    public:
        Counter() : Node("message_counter")
        {
            publisher_ = this->create_publisher<turtle_control::msg::MessageCounter>("count_number", 10);

            msg_counter.count = 0;
            msg_counter.state = false;
            msg_counter.name = "Robot";

            auto pub_count = [this]() -> void
            {
                if(msg_counter.count == 10)
                {
                    msg_counter.count = 0;
                    msg_counter.state = !msg_counter.state;
                    std::cout << "Hey " << msg_counter.name << "! Starting again." << std::endl;
                    std::cout << "State changed." << std::endl;
                }

                std::cout << "Counter is " << msg_counter.count << std::endl;
                std::cout << "State is " << std::boolalpha << msg_counter.state << std::endl;

                this->publisher_->publish(msg_counter);
                msg_counter.count++;
            };

            timer_ = this->create_wall_timer(1s, pub_count);
        }
    private:
        rclcpp::Publisher<turtle_control::msg::MessageCounter>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        turtle_control::msg::MessageCounter msg_counter;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Counter>());
    rclcpp::shutdown();

    return 0;
}