#include <iostream>		  // std::cout
#include <chrono>		  // std::chrono	
#include <thread>         // std::thread
#include <memory>
#include <vector>
#include <future>

//monoDrive Includes
#include "Simulator.h"
#include "Configuration.h"  // holder for sensor, simulator, scenario, weather, and vehicle configurations
#include "Sensor.h"
#include "sensor_config.h"
#include "Stopwatch.h"
#include "LaneSpline.h"
#include "DataFrame.h"

using namespace lane_spline;

LaneSpline lanespline;

std::vector<std::shared_ptr<Sensor>> create_sensors_for(const std::string& ip)
{
    std::vector<std::shared_ptr<Sensor>> sensors;
    CameraConfig fc_config;
    fc_config.server_ip = ip;
    fc_config.listen_port = 8100;
    fc_config.location.z = 200;
    fc_config.rotation.pitch = -5;
    fc_config.resolution = Resolution(512,512);

    sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(fc_config)));

    ViewportCameraConfig vp_config;
    vp_config.server_ip = ip;
    vp_config.location.z = 200;
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    StateConfig state_config;
    state_config.desired_tags = {"ego"};
    state_config.server_ip = ip;
    state_config.listen_port = 8101;
    state_config.debug_drawing = true;
    state_config.undesired_tags = {""};
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<StateConfig>(state_config)));

    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }
    return sensors;
}

void control_vehicle(Simulator& simulator, Sensor &sensor){
    auto frame = static_cast<StateFrame*>(sensor.frame);

    VehicleState* vehicle_frame = nullptr;
    for(auto& vehicle : frame->vehicles){
        for(auto& tag : vehicle.state.tags){
            if(tag == "ego"){
                vehicle_frame = &vehicle;
                break;
            }
        }
    }
    if(vehicle_frame == nullptr){
        throw std::runtime_error("Error, No ego was detected in tags.");
    }

    Eigen::VectorXd position(3);
    position << vehicle_frame->state.odometry.pose.position.x,
        vehicle_frame->state.odometry.pose.position.y,
        vehicle_frame->state.odometry.pose.position.z;
    Eigen::Quaternion<double> orientation(
        vehicle_frame->state.odometry.pose.orientation.w,
        vehicle_frame->state.odometry.pose.orientation.x,
        vehicle_frame->state.odometry.pose.orientation.y,
        vehicle_frame->state.odometry.pose.orientation.z
    );
    auto nearestIndex = lanespline.GetNearestPoint("road_0", "lane_2", position);
    auto& lane_points = lanespline.spline_map["road_0"]["lane_2"];
    int nextPointIndex = nearestIndex;
    if(nearestIndex >= lane_points.size()-4){
        nextPointIndex = (int)lane_points.size() - 1;
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
    
    simulator.sendControl(0.75f, 0.0f, 1, angle);
}

int main(int argc, char** argv)
{
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;
    lanespline = LaneSpline(std::string("examples/cpp/lane_follower/Straightaway5k.json"));
    
    //Read JSON files in cpp_client/config directory
    Configuration config(
        "examples/config/simulator_no_traffic.json",
        "examples/config/weather.json",
        "examples/config/scenario_config_single_vehicle.json");
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);
    sim0.configure();

    //Setup and Connect Sensors
    std::vector<std::shared_ptr<Sensor>> sensors = create_sensors_for(server0_ip);

    while(true){
        sim0.sampleAll(sensors);
        control_vehicle(sim0, *sensors[1]);
    }
    
    return 0;
}
