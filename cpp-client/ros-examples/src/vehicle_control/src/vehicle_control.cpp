#include <iostream>		  // std::cout
#include <chrono>		  // std::chrono	
#include <thread>         // std::thread
#include <memory>
#include <vector>
#include <future>
#include <chrono>

//monoDrive Includes
#include "LaneSpline.h"

#include "ros/ros.h"
#include "ros/package.h"
#include "monodrive_msgs/VehicleControl.h"
#include "monodrive_msgs/StateSensor.h"


using namespace lane_spline;
LaneSpline lanespline;

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

// LaneSpline lanespline;
std::string vehicle_name = "Ego";
std::shared_ptr<ros::NodeHandle> node_handle;

ros::Publisher vehicle_control_pub;
ros::Subscriber state_sensor_sub;

// Debug timing for measuing response rate
auto freq_time = std::chrono::high_resolution_clock::now();
int sample_count = 0;
const bool VERBOSE = false;

monodrive_msgs::StateSensor state_data;

void control_vehicle(){
    monodrive_msgs::VehicleState vs;
    for(auto& vehicle : state_data.vehicles){
        if(vehicle.name == "Ego"){
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
    msg.name = "Ego";
    msg.throttle = 0.75f;
    msg.brake = 0.f;
    msg.steer = angle;
    msg.drive_mode = 1;

    vehicle_control_pub.publish(msg);
}

void state_sensor_callback(const monodrive_msgs::StateSensor &state_sensor_msg){
    if(++sample_count >= 100 and VERBOSE) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_diff = 100.0 / 
            (std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - freq_time).count() / 1e9);
        std::cout << "Sample frequency (Hz): " << time_diff << std::endl;
        freq_time = current_time;
        sample_count = 0;
    }

    state_data = state_sensor_msg;
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
    state_sensor_sub = node_handle->subscribe("/monodrive/state_sensor", 5, &state_sensor_callback);

    ros::Rate rate(100);

    while(ros::ok()){
        control_vehicle();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
