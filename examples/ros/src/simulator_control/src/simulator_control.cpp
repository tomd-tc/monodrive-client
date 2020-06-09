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

std::vector<std::shared_ptr<Sensor>> create_sensors_for(const Simulator& sim0)
{
    std::vector<std::shared_ptr<Sensor>> sensors;

    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim0.getServerIp();
    vp_config.server_port = sim0.getServerPort();
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256,256);
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    //CameraConfig cam_config;
    //cam_config.server_ip = sim0.getServerIp();
    //cam_config.server_port = sim0.getServerPort();
    //cam_config.listen_port = 8100; // + i;
    //cam_config.location.z = 225;
    //cam_config.rotation.pitch = -5;
    //cam_config.resolution = Resolution(1920, 1080);
    //cam_config.ros.publish_to_ros = true;
    //cam_config.ros.advertise = true;
    //cam_config.ros.topic = "/monodrive/cam_config";
    //cam_config.ros.message_type = "monodrive_msgs/StateSensor";
    //sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(cam_config)));

    StateConfig state_config;
    state_config.server_ip = sim0.getServerIp();
    state_config.server_port = sim0.getServerPort();
    state_config.desired_tags = {"vehicle", "ego"};
    state_config.listen_port = 8101;
    state_config.debug_drawing = false;
    state_config.undesired_tags = {"static"};
    state_config.ros.publish_to_ros = true;
    state_config.ros.advertise = true;
    state_config.ros.topic = "/monodrive/state_sensor";
    state_config.ros.message_type = "monodrive_msgs/StateSensor";
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<StateConfig>(state_config)));

    std::cout<<"***********ALL SENSOR CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }
    return sensors;
}

void run_monodrive(float fps, Simulator& sim){
    std::cout << "Creating Sensors." << std::endl;
    std::vector<std::shared_ptr<Sensor>> sensors = create_sensors_for(sim);
    std::cout << "Sensors configured!" << std::endl;

    ros::Rate rate(fps);

    while(ros::ok()){
        // Sample the sensors
        //sim.sample_all(sensors);
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
        configPath / "weather.json",
        configPath / "scenario.json");
    
    std::string server0_ip = config.simulator["server_ip"];
    int server_port = config.simulator["server_port"];
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()) {
        std::cerr << "Unable to configure simulator!" << std::endl;
        return -1;
    }

    /// initialize the vehicle
    float fps = 60.f;
    run_monodrive(fps, sim0);

    return 0;
}
