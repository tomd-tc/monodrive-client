#include <iostream>		  // std::cout
#include <chrono>		  // std::chrono	
#include <thread>         // std::thread
#include <memory>
#include <vector>

//monoDrive Includes
#include "Simulator.h"
#include "Configuration.h"  // holder for sensor, simulator, scenario, weather, and vehicle configurations
#include "Sensor.h"
#include "sensor_config.h"
#include "Stopwatch.h"
#include "LaneSpline.h"
#include <future>

using namespace lane_spline;

LaneSpline lanespline;

std::vector<Sensor> create_sensors_for(const std::string& ip)
{
    std::vector<Sensor> sensors;
    CameraConfig fc_config;// = *(new CameraConfig());
    fc_config.server_ip = ip;
    fc_config.listen_port = 8100;
    fc_config.location.z = 200;
    fc_config.rotation.pitch = -5;
    fc_config.resolution = CameraConfig::Resolution(512,512);

    sensors.emplace_back(fc_config);

    ViewportCameraConfig vp_config;
    vp_config.server_ip = ip;
    vp_config.location.z = 200;
    Sensor(vp_config).configure();

    StateConfig state_config;
    state_config.desired_tags = {"vehicle", "ego"};
    state_config.server_ip = ip;
    state_config.listen_port = 8101;
    state_config.debug_drawing = true;
    state_config.undesired_tags = {""};
    sensors.emplace_back(state_config);

    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor.configure();
    }
    return sensors;
}

void control_vehicle(Simulator& simulator, Sensor &sensor){
    std::string json_string(reinterpret_cast<char*>(sensor.recvBuffer.data()), sensor.recvBuffer.size());
    nlohmann::json frames = json::parse(json_string);

    nlohmann::json vehicle_frame;
    nlohmann::json stop_sign_frame;
    for(auto& f : frames["frame"]){
        for(auto& tag : f["tags"]){
            if(tag == "ego"){
                vehicle_frame = f;
                break;
            }
        }
    }

    Eigen::VectorXd position(3);
    position << vehicle_frame["position"][0].get<double>(),
        vehicle_frame["position"][1].get<double>(),
        vehicle_frame["position"][2].get<double>();
    Eigen::Quaternion<double> orientation(
        vehicle_frame["orientation"][3].get<float>(),
        vehicle_frame["orientation"][0].get<float>(),
        vehicle_frame["orientation"][1].get<float>(),
        vehicle_frame["orientation"][2].get<float>());
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
    
    nlohmann::json msg;
    msg["forward_amount"] = 0.75f;
    msg["brake_amount"] = 0.0f;
    msg["drive_mode"] = 1;
    msg["right_amount"] = angle;

    simulator.send_command(ApiMessage(777, EgoControl_ID, true, msg));
}

int main(int argc, char** argv)
{
    //Single Simulator Example
    string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;
    
    //Read JSON files in cpp_client/config directory
    Configuration config(
        "cpp-examples/lane_follower/simulator.json",
        "simulator-cpp-client/config/vehicle.json",
        "simulator-cpp-client/config/weather.json",
        "cpp-examples/lane_follower/scenario.json");
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);
    sim0.configure();
    lanespline = LaneSpline("cpp-examples/lane_follower/Straightaway5k.json");

    //Setup and Connect Sensors
    std::vector<Sensor> sensors = create_sensors_for(server0_ip);
    
    //Get number of steps in scenario and start timer
    int nSteps = config.scenario.size();

    /// initialize the vehicle, the first control command spawns the vehicle
    sim0.send_command(ApiMessage(123, EgoControl_ID, true, 
        {   {"forward_amount", 0.5}, 
            {"right_amount", 0.0},
            {"brake_amount", 0.0},
            {"drive_mode", 1}
        }));
    //Step through scenario while reading sensor ouputs
    std::future<bool> task;
    while(true){
        task = std::async([&sim0](){
            return sim0.send_command(ApiMessage(999, SampleSensorsCommand_ID, true, {}));
        });
        //sample all sensors
        for(auto& sensor : sensors){
            sensor.sample();
        }
        if(!task.get()){
            break;
        }
        else{
            control_vehicle(sim0, sensors[1]);
        }
    }
    
    return 0;
}
