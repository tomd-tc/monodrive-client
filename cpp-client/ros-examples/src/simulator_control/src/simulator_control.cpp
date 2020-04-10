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
#include <future>

#include "ros/ros.h"
#include "ros/package.h"

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

std::string vehicle_name;

std::vector<Sensor> create_sensors_for(const std::string& ip)
{
    std::vector<Sensor> sensors;

    ViewportCameraConfig vp_config;
    vp_config.server_ip = ip;
    vp_config.location.z = 200;
    Sensor(vp_config).configure();

    StateConfig state_config;
    state_config.desired_tags = {"vehicle", "ego"};
    state_config.server_ip = ip;
    state_config.listen_port = 8101;
    state_config.debug_drawing = false;
    state_config.undesired_tags = {""};

    state_config.ros.publish_to_ros = true;
    state_config.ros.advertise = true;
    state_config.ros.topic = "/monodrive/state_sensor";
    state_config.ros.message_type = "monodrive_msgs/StateSensor";
    sensors.emplace_back(state_config);

    std::cout<<"***********ALL SENSOR CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor.configure();
    }
    return sensors;
}

void run_monodrive(float fps, Simulator& sim){
    ros::Rate rate(fps);
    // mono::precise_stopwatch stopwatch;
    while(ros::ok()){
        sim.send_command(ApiMessage(999, SampleSensorsCommand_ID, true, {}));
        // ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_follower");
    ros::NodeHandle n;
    fs::path path(ros::package::getPath("simulator_control"));
    fs::path configPath = path / "config";

    //Read JSON files in cpp_client/config directory
    Configuration config(
        configPath / "simulator.json",
        configPath / "vehicle.json",
        configPath / "weather.json",
        configPath / "scenario.json");
    
    vehicle_name = config.vehicle["name"];
    std::string server0_ip = config.simulator["server_ip"];
    int server_port = config.simulator["server_port"];

    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);
    sim0.configure();

    //Setup and Connect Sensors
    std::cout << "Creating Sensors." << std::endl;
    std::vector<Sensor> sensors = create_sensors_for(server0_ip);

    /// initialize the vehicle
    std::cout << "Spawning vehicle." << std::endl;
    if(!sim0.send_command(ApiMessage(777, SpawnVehicleCommand_ID, true, config.vehicle)))
        return 0;

    float fps = 60.f;
    run_monodrive(fps, sim0);

    return 0;
}
