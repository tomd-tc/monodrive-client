
#include <iostream>		  // std::cout

//monoDrive Includes
#include "Simulator.h"
#include "Configuration.h"  // holder for sensor, simulator, scenario, weather, and vehicle configurations
#include "Sensor.h"
#include "sensor_config.h"
#include "Stopwatch.h"


int main(int argc, char** argv)
{
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;

    //Read JSON files in cpp_client/config directory
    Configuration config(
        "examples/config/simulator_straightaway.json",
        "examples/config/weather.json",
        "examples/config/scenario_multi_vehicle_straightaway.json"
    );
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;
    for(int i = 0; i < 4; i++) 
    {
        RPMConfig r_config;
        r_config.server_ip = sim0.getServerIp();
        r_config.server_port = sim0.getServerPort();
        r_config.listen_port = 8100 + i;
        r_config.wheel_number = i;
        sensors.push_back(std::make_shared<Sensor>(std::make_unique<RPMConfig>(r_config)));
    }

    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim0.getServerIp();
    vp_config.server_port = sim0.getServerPort();
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256,256);
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    // Send configurations to the simulator
    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }

    std::mutex rpmDisplayMutex;
    auto rpmCallback = [&rpmDisplayMutex](DataFrame * frame)
    {
        std::lock_guard<std::mutex> lock(rpmDisplayMutex);
        auto &rpmFrame = *static_cast<RPMFrame *>(frame);
        std::cout << "Wheel: " << rpmFrame.wheel_number << " Speed: " << rpmFrame.speed << std::endl;
    };

    for(auto& sensor : sensors) {
        sensor->sampleCallback = rpmCallback;
    }

    std::cout << "Sampling sensor loop" << std::endl;

    EgoControlConfig ego_control_config;
    ego_control_config.forward_amount = 0.5f;
    ego_control_config.right_amount = 0.0f;
    ego_control_config.brake_amount = 0.0f;
    ego_control_config.drive_mode = 1;

    while(true)
    {
        sim0.sendCommand(ego_control_config.message());
        sim0.sampleAll(sensors);
    }
    
    return 0;
}