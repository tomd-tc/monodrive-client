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

std::vector<Sensor> create_sensors_for(const std::string& ip)
{
    std::vector<Sensor> sensors;
    CameraConfig fc_config;// = *(new CameraConfig());
    fc_config.server_ip = ip;
    fc_config.listen_port = 8100;
    fc_config.location.z = 200;
    fc_config.rotation.pitch = -5;
    fc_config.resolution = CameraConfig::Resolution(1024,1024);

    sensors.emplace_back(fc_config);

    ViewportCameraConfig vp_config;
    vp_config.server_ip = ip;
    vp_config.location.z = 200;
    Sensor(vp_config).configure();


    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor.configure();
    }
    return sensors;
}

int main(int argc, char** argv)
{
    //Single Simulator Example
    string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;
    
    //Read JSON files in cpp_client/config directory
    Configuration config(
        "simulator-cpp-client/config/simulator.json",
        "simulator-cpp-client/config/vehicle.json",
        "simulator-cpp-client/config/weather.json",
        "simulator-cpp-client/config/scenario.json"
    );
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    //Setup and Connect Sensors
    std::vector<Sensor> sensors = create_sensors_for(server0_ip);
    
    //Get number of steps in scenario and start timer
    int nSteps = config.scenario.size();
    int idx = 0;
    mono::precise_stopwatch stopwatch;

    //Step through scenario while reading sensor ouputs
    std::future<bool> task;
    for(; idx < nSteps; idx++)
    {	
        //step simulator
        task = std::async([&sim0, &idx](){
            return sim0.step(idx, 1);
        });
        //sample all sensors
        for(auto& sensor : sensors)
        {
            sensor.sample();
        }
        if(!task.get()){
            break;
        }
    }
    //Calculate FPS
    auto scenario_time = stopwatch.elapsed_time<unsigned int, std::chrono::microseconds>();
    auto fps = float(idx+1)/scenario_time*1000000.0;
    std::cout<< "fps = " + std::to_string(fps) << std::endl;
    
    return 0;
}
