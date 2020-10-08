#include <iostream>		  // std::cout
#include <chrono>		  // std::chrono	
#include <thread>         // std::thread
#include <memory>
#include <vector>
#include <future>

//monoDrive Includes
#include "Simulator.h"
#include "Configuration.h"  // holder for sensor, simulator, scenario, weather, and vehicle configurations
#include "MessageFactory.h"

#include "ros/ros.h"
#include "ros/package.h"
#include "monodrive_msgs/VehicleControl.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

std::string vehicle_name = "subcompact_monoDrive_01";
std::shared_ptr<ros::NodeHandle> node_handle;

ros::Publisher vehicle_control_pub;
ros::Subscriber joy_sub_;


std::vector<float> wheel_data = {0,0,0,0};


void control_vehicle_ros(){
    monodrive_msgs::VehicleControl msg;
    msg.name = vehicle_name;
    msg.throttle = wheel_data[0];
    msg.brake = wheel_data[1];
    msg.steer = wheel_data[2];
    msg.drive_mode = wheel_data[3];
    
    vehicle_control_pub.publish(msg);
}

void control_vehicle_tcp(Simulator* sim){
    EgoControlConfig egoControl;
    egoControl.forward_amount = wheel_data[0];
    egoControl.brake_amount = wheel_data[1];
    egoControl.right_amount = wheel_data[2];
    egoControl.drive_mode = wheel_data[3];
    
    sim->sendCommand(ApiMessage(777, EgoControl_ID, true, egoControl.dump()));
}


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  wheel_data[0] = (joy->axes[1] == -1) ? 0 : joy->axes[1]; //throttle
  wheel_data[1] = (joy->axes[2] == -1) ? 0 : joy->axes[2]; //brake
  wheel_data[2] = -1.0 * joy->axes[0]; // steering - Need to inverse to match movement
  wheel_data[3] =  (joy->buttons[8]) == 0.0 ? 1.0 : -1.0; //drive_mode
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheel_vehicle_control");

    bool use_simulator_ros = true;
    ros::NodeHandle nh("~");
    nh.getParam("use_simulator_ros", use_simulator_ros);

    Simulator* sim = nullptr;
    if (!use_simulator_ros) {
        fs::path path(ros::package::getPath("simulator_control"));
        fs::path configPath = path / "config";
        Configuration config(configPath / "simulator.json");

        std::string server0_ip = config.simulator["server_ip"];
        int server_port = config.simulator["server_port"];
        sim = &Simulator::getInstance(server0_ip, server_port);
    }

    // create vehicle controller publisher and sensor subscriber
    node_handle = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
    vehicle_control_pub = node_handle->advertise<monodrive_msgs::VehicleControl>("/monodrive/vehicle_control", 1);
    joy_sub_ = node_handle->subscribe("/joy", 1, &joyCallback);

    ros::Rate rate(100);

    while(ros::ok()){
        if (use_simulator_ros) {
            control_vehicle_ros();
        } else {
            control_vehicle_tcp(sim);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
