#include <iostream>		  // std::cout
#include <chrono>		  // std::chrono	
#include <thread>         // std::thread
#include <memory>
#include <vector>
#include <future>

//monoDrive Includes

#include "ros/ros.h"
#include "ros/package.h"
#include "monodrive_msgs/VehicleControl.h"
#include "monodrive_msgs/StateSensor.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

std::string vehicle_name = "crossover_monoDrive_01";
std::shared_ptr<ros::NodeHandle> node_handle;

ros::Publisher vehicle_control_pub;
ros::Subscriber state_sensor_sub;
ros::Subscriber joy_sub_;


monodrive_msgs::StateSensor state_data;
std::vector<float> wheel_data = {0,0,0,0};


void control_vehicle(){
   
    monodrive_msgs::VehicleControl msg;
    msg.name = vehicle_name;
    msg.throttle = wheel_data[0];
    msg.brake = wheel_data[1];
    msg.steer = wheel_data[2];
    msg.drive_mode = wheel_data[3];
    
    vehicle_control_pub.publish(msg);
}

void state_sensor_callback(const monodrive_msgs::StateSensor &state_sensor_msg){
    state_data = state_sensor_msg;
    for(auto& vehicle : state_data.vehicles) {
        for(auto& tag : vehicle.tags) {
            if(tag == "ego") {
                vehicle_name = vehicle.name;
            }
        }
    }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  wheel_data[0] = (joy->axes[1] == -1) ? 0 : joy->axes[1]; //throttle
  wheel_data[1] = (joy->axes[2] == -1) ? 0 : joy->axes[2]; //brake
  wheel_data[2] = -1.0 * joy->axes[0]; // steering - Need to inverse to match movement
  wheel_data[3] =  (joy->buttons[8]) == 0.0 ? 1.0 : -1.0; //drive_mode

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheel_vehicle_control");
    std::cout << "wheel_vehicle_control! " << std::endl;

    fs::path path(ros::package::getPath("wheel_vehicle_control"));
    fs::path configPath = path / "config";

    // create vehicle controller publisher and sensor subscriber
    node_handle = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
    vehicle_control_pub = node_handle->advertise<monodrive_msgs::VehicleControl>("/monodrive/vehicle_control", 1);
    state_sensor_sub = node_handle->subscribe("/monodrive/state_sensor", 1, 
        &state_sensor_callback);
    joy_sub_ = node_handle->subscribe("/joy", 1, &joyCallback);

    ros::Rate rate(100);

    while(ros::ok()){
        control_vehicle();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
