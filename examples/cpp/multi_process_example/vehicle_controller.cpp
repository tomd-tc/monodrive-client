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

LaneSpline lanespline = LaneSpline(std::string("examples/cpp/multi_process_example/Straightaway5k.json"));

//Single Simulator Example
std::string server0_ip = "127.0.0.1";
int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;

EgoControlConfig planning(DataFrame* dataFrame){
    auto& frame = *static_cast<StateFrame*>(dataFrame);
    std::cout << "sample, game, wall " << frame.sample_count << " " << frame.game_time << " " << frame.wall_time << std::endl;

    VehicleState* vehicle_frame = nullptr;
    for(auto& vehicle : frame.vehicles){
        for(auto& tag : vehicle.state.tags){
            if(tag == "ego"){
                vehicle_frame = &vehicle;
                break;
            }
        }
    }
    if(vehicle_frame == nullptr){
        std::cout << "No ego vehicle in frame." << std::endl;
        return EgoControlConfig();
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

    EgoControlConfig egoControl;
    egoControl.forward_amount = 0.75;
    egoControl.brake_amount = 0.0;
    egoControl.drive_mode = 1;
    egoControl.right_amount = (float)angle;
    return egoControl;
}

int main(int argc, char** argv)
{
    Simulator& sim0 = Simulator::getInstance(server0_ip, server_port);
    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;

    StateConfig state_config;
    state_config.desired_tags = {"vehicle"};
    state_config.undesired_tags = {"static"};
    state_config.server_ip = sim0.getServerIp();
    state_config.server_port = sim0.getServerPort();
    state_config.listen_port = 8200;
    state_config.debug_drawing = true;
    state_config.undesired_tags = {""};
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<StateConfig>(state_config)));

    // Send configurations to the simulator and start listening
    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }

    // start our main perception planning and control loop
    std::cout << "Sampling sensor loop" << std::endl;
    while(true)
    {
        mono::precise_stopwatch watch;
        // perception
        sim0.sampleSensorList(sensors);
        // planning
        EgoControlConfig egoControl = planning(sensors[0]->frame);
        // control
        sim0.sendCommand(ApiMessage(777, EgoControl_ID, true, egoControl.dump()));
        std::cout << watch.elapsed_time<unsigned int, std::chrono::milliseconds>() << " (ms)" << std::endl;
    }
    
    return 0;
}