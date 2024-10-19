#include <rclcpp/rclcpp.hpp>
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
        
        // Create a timer to periodically check for messages
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&YarpReceiver::receive_message, this)
        );
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
        }
    }

    yarp::os::Port port;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YarpReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}