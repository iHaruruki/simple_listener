#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>

class YarpReceiver : public rclcpp::Node
{
public:
    YarpReceiver()
    : Node("yarp_receiver")
    {
        RCLCPP_INFO(this->get_logger(), "YARP Receiver Node has been started.");
        
        // Initialize YARP network
        yarp::os::Network yarp;
        
        // Open YARP port
        port.open("/receiver");

        // Connect to the sender port
        yarp::os::Network::connect("/sender", "/receiver");
        
        // Create a timer to periodically check for messages
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&YarpReceiver::receive_message, this)
        );

        // Initialize ROS publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("yarp_to_ros", 10);
    }

    ~YarpReceiver()
    {
        port.close();
    }

private:
    void receive_message()
    {
        yarp::os::Bottle bot;
        if (port.read(bot))
        {
            RCLCPP_INFO(this->get_logger(), "Received message: %s", bot.toString().c_str());

            // Publish the received message to ROS topic
            auto message = std_msgs::msg::String();
            message.data =  bot.toString();
            publisher_->publish(message);
        }
    }

    yarp::os::Port port;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YarpReceiver>();

    rclcpp::Rate rate(10);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}