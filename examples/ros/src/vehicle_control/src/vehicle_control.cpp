#include <iostream>		  // std::cout
#include <chrono>		  // std::chrono	
#include <thread>         // std::thread
#include <memory>
#include <vector>
#include <future>

//monoDrive Includes
#include "LaneSpline.h"

#include "ros/ros.h"
#include "ros/package.h"
#include "monodrive_msgs/VehicleControl.h"
#include "monodrive_msgs/StateSensor.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

using namespace lane_spline;
LaneSpline lanespline;

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

// LaneSpline lanespline;
std::string vehicle_name = "crossover_monoDrive_01";
std::shared_ptr<ros::NodeHandle> node_handle;

ros::Publisher vehicle_control_pub;
ros::Subscriber state_sensor_sub;
ros::Subscriber joy_sub_;


monodrive_msgs::StateSensor state_data;
std::vector<float> wheel_data = {0,0,0,0};



void control_vehicle(){
    monodrive_msgs::VehicleState vs;
    for(auto& vehicle : state_data.vehicles){
        if(vehicle.name == vehicle_name) {
            vs = vehicle;
        }
    }
    Eigen::VectorXd position(3);
    position << vs.odometry.pose.pose.position.x,
        vs.odometry.pose.pose.position.y,
        vs.odometry.pose.pose.position.z;
    Eigen::Quaternion<double> orientation(
        vs.odometry.pose.pose.orientation.w,
        vs.odometry.pose.pose.orientation.x,
        vs.odometry.pose.pose.orientation.y,
        vs.odometry.pose.pose.orientation.z
    );

    auto nearestIndex = lanespline.GetNearestPoint("road_0", "lane_2", position);
    auto& lane_points = lanespline.spline_map["road_0"]["lane_2"];
    int nextPointIndex = nearestIndex;
    if(nearestIndex >= lane_points.size()-4){
        nextPointIndex = lane_points.size()-1;
    }
    else{
        nextPointIndex += 3;
    }
    Eigen::VectorXd forwardVector(3);
    forwardVector << 1, 0, 0;
    forwardVector = orientation * forwardVector;
    auto nextPoint = lane_points[nextPointIndex];
    Eigen::VectorXd dirToNextPoint = nextPoint - position;
    dirToNextPoint.normalize();

    double angle = -dirToNextPoint.head<3>().cross(forwardVector.head<3>())[2];

    monodrive_msgs::VehicleControl msg;
    msg.name = vehicle_name;
    std::cout << "throttle: " <<wheel_data[0] <<"\t"<<"brake: " <<wheel_data[1] << "\t"<< "steer: " <<wheel_data[2] << "\t" <<  "drive_mode: " << wheel_data[3] <<std::endl;
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
//   std::cout<<"AXE[0]: "<< joy->axes[0] <<std::endl;
  /*
  0 - throttle
  1 - brake
  2 - steer
  3 - drive_mode
  */ 

  float throttle =  (joy->axes[1] == -1) ? 0 : joy->axes[1];
  float brake =  (joy->axes[2] == -1) ? 0 : joy->axes[2];
  float steer =  float(joy->axes[0]);
  float drive_mode =  (joy->buttons[8]) == 0.0 ? 1.0 : -1.0;


  wheel_data[0] = throttle;
  wheel_data[1] = brake;
  wheel_data[2] = -1.0*steer;
  wheel_data[3] = drive_mode;
//   std::cout << "throttle: " <<wheel_data[0] <<"\t"<<"brake: " <<wheel_data[1] << "\t"<< "steer: " <<wheel_data[2] << "\t" <<  "drive_mode: " << wheel_data[3] <<std::endl;


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_control");

    // read in local map
    fs::path path(ros::package::getPath("vehicle_control"));
    fs::path configPath = path / "config";
    lanespline = LaneSpline((configPath / "Straightaway5k.json").string());

    // create vehicle controller publisher and sensor subscriber
    node_handle = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
    vehicle_control_pub = node_handle->advertise<monodrive_msgs::VehicleControl>("/monodrive/vehicle_control", 1);
    state_sensor_sub = node_handle->subscribe("/monodrive/state_sensor", 1, 
        &state_sensor_callback);
    joy_sub_ = node_handle->subscribe("/joy", 10, &joyCallback);

    ros::Rate rate(100);

    while(ros::ok()){
        control_vehicle();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
