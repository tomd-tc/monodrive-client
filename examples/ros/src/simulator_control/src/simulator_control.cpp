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

    StateConfig state_config;
    state_config.server_ip = sim0.getServerIp();
    state_config.server_port = sim0.getServerPort();
    state_config.desired_tags = {"ego"};
    state_config.listen_port = 8101;
    state_config.debug_drawing = false;
    state_config.undesired_tags = {"static"};
    state_config.ros.publish_to_ros = true;
    state_config.ros.advertise = true;
    state_config.ros.topic = "/monodrive/state_sensor";
    state_config.ros.message_type = "monodrive_msgs/StateSensor";
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<StateConfig>(state_config)));

    WaypointConfig waypoint_config;
    waypoint_config.server_ip = sim0.getServerIp();
    waypoint_config.server_port = sim0.getServerPort();
    waypoint_config.distance = 1000.0;
    waypoint_config.frequency = 100.0;
    waypoint_config.listen_port = 8201;
    waypoint_config.ros.publish_to_ros = true;
    waypoint_config.ros.advertise = true;
    waypoint_config.ros.topic = "/monodrive/waypoint_sensor";
    waypoint_config.ros.message_type = "monodrive_msgs/WaypointSensor";
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<WaypointConfig>(waypoint_config)));

    IMUConfig imu_config;
    imu_config.server_ip = sim0.getServerIp();
    imu_config.server_port = sim0.getServerPort();
    imu_config.listen_port = 8102;
    imu_config.ros.publish_to_ros = true;
    imu_config.ros.advertise = true;
    imu_config.ros.topic = "/monodrive/imu_sensor";
    imu_config.ros.message_type = "sensor_msgs/Imu";
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<IMUConfig>(imu_config)));

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
        configPath / "simulator_infinity.json",
        configPath / "weather.json",
        configPath / "scenario_correct_map.json");
    
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
