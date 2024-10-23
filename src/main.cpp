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
#include <iostream>

int main(int argc, char * argv[])
{
	yarp::os::Network yarp;

	// Open YARP port
	yarp::os::BufferedPort<yarp::os::Bottle> p_cmd; //motor command
	yarp::os::BufferedPort<yarp::os::Bottle> p_enc; //encoder reading
	p_cmd.open("/remoteController/command:o");   //motor command
	p_enc.open("/remoteController/encoder:i");   //encoder reading

    // Connect to the sender port
    yarp::os::Network::connect("/remoteController/command:o","/vehicleDriver/remote:i");  //motor command
    yarp::os::Network::connect("/vehicleDriver/encoder:o", "/remoteController/encoder:i");  //encoder reading

   	while(true){
        yarp::os::Bottle* bt = p_enc.read(false);
        std::vector<double> enc(4);
        if (bt != nullptr) {
            for(int i = 0; i < enc.size(); i++){
                enc[i] = bt->get(i).asFloat64();
            }
        }

        std::vector<double> cmd(4);
        cmd[0]=0.05;
        cmd[1]=0.00;
        cmd[2]=0.00;
        cmd[3]=0.00;
        yarp::os::Bottle& bc = p_cmd.prepare();
        bc.clear();
        for(int i = 0; i < cmd.size(); i++){
            bc.addFloat64(cmd[i]);
        }
            p_cmd.write();

        std::cout << "enc " 
            << enc[0] << " " 
            << enc[1] << " " 
            << enc[2] << " " 
            << enc[3] << " " 
            << "cmd " 
            << cmd[0] << " " 
            << cmd[1] << " " 
            << cmd[2] << " " 
            << cmd[3] << std::endl;
        yarp::os::Time::delay(0.05);
    }	

    p_cmd.close();
    p_enc.close();
	return 0;
}
