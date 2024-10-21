#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yarp/os/all.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>

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
        p_tou.open("/remoteController/remote:i");   //encoder reading

        // Connect to the sender port
        yarp::os::Network::connect("/sender", "/receiver");
        yarp::os::Network::connect("/vehicleDriver/remote:i", "/remoteController/remote:i");  //motor command
        yarp::os::Network::connect("/vehicleDriver/encoder:o", "/remoteController/remote:o");  //encoder reading
        
        // Create a timer to periodically check for messages
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&YarpReceiver::receive_message, this)
        );

        // Initialize ROS publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("yarp_to_ros", 50);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 50); //odometry
        // cmd_vel subscriber
        cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 50, std::bind(&YarpReceiver::cmd_callback, this, std::placeholders::_1)
        );
    }

    /*~YarpReceiver()
    {
        port.close();
        p_cmd.close();
        p_tou.close();
    }*/

    void close_ports()
    {
        port.close();
        p_cmd.close();
        p_tou.close();
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

    void receive_odometry()
    {
        yarp::os::Bottle* bt = p_tou.read(false);
        if (bt != nullptr)
        {
            std::vector<double> tou(4);
            for(int i = 0; i < tou.size(); i++)
            {
                tou[i] = bt->get(i).asFloat64();
            }
            // Extract the values from the bottle
            double vx = tou[0];
            double vy = tou[1];
            double w  = tou[2];
            double ta = tou[3];

            // Publish the odometry message
            auto odom = nav_msgs::msg::Odometry();
            odom.pose.pose.position.x = vx;
            odom.pose.pose.position.y = vy;
            odom.pose.pose.orientation.z = w;
            //odom.pose.pose.orientation.w = ta;
            odom_publisher_->publish(odom);
        }
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
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;      //odometry
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_; //cmd_vel
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YarpReceiver>();

    rclcpp::Rate rate(20);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    node->close_ports();
    rclcpp::shutdown();
    return 0;
}