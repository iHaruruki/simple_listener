#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yarp/os/all.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

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
        p_cmd.open("/remoteController/remote:o");   //motor command
        //_tou.open("/remoteController/remote:i");   //encoder reading

        // Connect to the sender port
        yarp::os::Network::connect("/sender", "/receiver");
        yarp::os::Network::connect("/vehicleDriver/remote:i", "/remoteController/remote:i");  //motor command
        //yarp::os::Network::connect("/remoteController/remote:o", "/receiver");  //encoder reading
        
        // Create a timer to periodically check for messages
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&YarpReceiver::receive_message, this)
        );

        // Initialize ROS publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("yarp_to_ros", 10);
        cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&YarpReceiver::cmd_callback, this, std::placeholders::_1)
        );
    }

    ~YarpReceiver()
    {
        port.close();
        p_cmd.close();
        //_tou.close();
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

        yarp::os::Bottle* bc = p_cmd.read();
        
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Extract the required values
        double vx = msg->linear.x;
        double vy = 0;  //msg->linear.y;
        double bw = msg->angular.z;
        double ta = 0;  //msg->angular.x;

        // Create a vector with the extracted values
        std::vector<double> cmd = {vx, vy, bw, ta};

        // Prepare the YARP bottle
        yarp::os::Bottle& bc = p_cmd.prepare();
        bc.clear();
        for (int i = 0; i < cmd.size(); i++)
        {
            bc.addFloat64(cmd[i]);
        }
        p_cmd.write();
    }

    yarp::os::Port port;
    yarp::os::BufferedPort<yarp::os::Bottle> p_cmd; //motor command
    yarp::os::BufferedPort<yarp::os::Bottle> p_tou; //encoder reading

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
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