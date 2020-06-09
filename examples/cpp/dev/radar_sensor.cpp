#include <iostream>		  // std::cout
#include <chrono>		  // std::chrono	
#include <thread>         // std::thread
#include <memory>
#include <vector>
#include <future>
#include <thread>

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
        "examples/config/scenario_config_single_vehicle.json"
    );
    // config.simulator["map"] = "RadarCube";
    // config.scenario["frame"][0]["position"] = {0.f, 0.f, 20.f};
    // config.scenario["frame"][0]["velocity"] = {0.f, 0.f, 0.f};

    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;

    RadarConfig r_config;
    r_config.location.x = 300.f;
    r_config.location.z = 50.f;
    r_config.paint_targets = false;
    r_config.send_radar_cube = true;
    r_config.max_radar_returns = 45;
    r_config.sbr.ray_division_y = 10.f;
    r_config.sbr.ray_division_z = 10.f;
    r_config.sbr.debug_scan = false;
    r_config.sbr.debug_rescan = false;
    r_config.sbr.debug_frustum = false;
    r_config.server_ip = sim0.getServerIp();
    r_config.server_port = sim0.getServerPort();
    r_config.listen_port = 8102;
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<RadarConfig>(r_config)));

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

    for(auto& sensor : sensors){
        sensor->startSampleLoop();
    }

    sensors[0]->sampleCallback = [](DataFrame* frame){
        auto& radarFrame = *static_cast<RadarFrame*>(frame);
        std::cout << nlohmann::json(radarFrame.radarTargetListFrame->targets).dump() << std::endl;
        std::cout << nlohmann::json(radarFrame.radarTargetListFrame->gt_targets).dump() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    };

    std::cout << "Sampling sensor loop" << std::endl;
    int count = 0;
    while(true)
    {	
        sim0.sampleAll(sensors);
    }
    
    return 0;
}
