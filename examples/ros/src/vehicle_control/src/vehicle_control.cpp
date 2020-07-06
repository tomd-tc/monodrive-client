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
#include "sensor_msgs/Imu.h"
#include "monodrive_msgs/VehicleControl.h"
#include "monodrive_msgs/StateSensor.h"
#include "monodrive_msgs/WaypointSensor.h"

using namespace lane_spline;
LaneSpline lanespline;

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

// LaneSpline lanespline;
std::string vehicle_name = "subcompact_monoDrive_01";
std::shared_ptr<ros::NodeHandle> node_handle;

ros::Publisher vehicle_control_pub;
ros::Subscriber state_sensor_sub;
ros::Subscriber wp_sensor_sub;
ros::Subscriber imu_sensor_sub;

monodrive_msgs::StateSensor state_data;
monodrive_msgs::WaypointSensor waypoint_data;
sensor_msgs::Imu imu_data;

void control_vehicle(){
    monodrive_msgs::VehicleState vs;
    for(auto& vehicle : state_data.vehicles){
        if(vehicle.name == vehicle_name) {
            vs = vehicle;
        }
    }
    Eigen::VectorXd position(3);
    position << vs.pose.pose.position.x,
        vs.pose.pose.position.y,
        vs.pose.pose.position.z;
    Eigen::Quaternion<double> orientation(
        vs.pose.pose.orientation.w,
        vs.pose.pose.orientation.x,
        vs.pose.pose.orientation.y,
        vs.pose.pose.orientation.z
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
    msg.throttle = 0.75f;
    msg.brake = 0.f;
    msg.steer = angle;
    msg.drive_mode = 1;

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

void waypoint_sensor_callback(const monodrive_msgs::WaypointSensor &waypoint_sensor_msg) {
    waypoint_data = waypoint_sensor_msg;
}

void imu_sensor_callback(const sensor_msgs::Imu &imu_sensor_msg) {
    imu_data = imu_sensor_msg;
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
    wp_sensor_sub = node_handle->subscribe("/monodrive/waypoint_sensor", 1, 
        &waypoint_sensor_callback);
    imu_sensor_sub = node_handle->subscribe("/monodrive/imu_sensor", 1, 
        &imu_sensor_callback);

    ros::Rate rate(100);

    while(ros::ok()){
        control_vehicle();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
