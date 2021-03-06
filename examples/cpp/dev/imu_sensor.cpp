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
    IMUConfig i_config;
    i_config.server_ip = sim0.getServerIp();
    i_config.server_port = sim0.getServerPort();
    i_config.listen_port = 8102;
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<IMUConfig>(i_config)));

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

    //Get number of steps in scenario and start timer
    mono::precise_stopwatch stopwatch;

    /// initialize the vehicle, the first control command spawns the vehicle
    sim0.sendControl(0.1, 0.0, 0.0, 1);

    sensors[0]->sampleCallback = [](DataFrame* frame){
        auto& imuFrame = *static_cast<ImuFrame*>(frame);
        std::cout << imuFrame.acceleration.x << " "
            << imuFrame.acceleration.y << " "
            << imuFrame.acceleration.z << " "
            << imuFrame.angular_velocity.x << " "
            << imuFrame.angular_velocity.y << " "
            << imuFrame.angular_velocity.z << std::endl;
    };

    std::cout << "Sampling sensor loop" << std::endl;
    while(true)
    {
        sim0.sampleAll(sensors);
    }
    
    return 0;
}
