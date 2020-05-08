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

std::vector<std::shared_ptr<Sensor>> create_sensors_for(const Simulator& sim0)
{
    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;
    RadarConfig r_config;
    r_config.location.x = 300.f;
    r_config.location.z = 50.f;
    r_config.paint_targets = true;
    r_config.send_radar_cube = true;
    r_config.server_ip = sim0.getServerIp();
    r_config.server_port = sim0.getServerPort();
    r_config.listen_port = 8102;
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<RadarConfig>(r_config)));

    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim0.getServerIp();
    vp_config.server_port = sim0.getServerPort();
    vp_config.location.z = 200;
    vp_config.resolution = ViewportCameraConfig::Resolution(256,256);
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    // Send configuraitons to the simulator
    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }
    return sensors;
}

void state_test(Simulator& sim0){
    //Setup and Connect Sensors
    std::vector<std::shared_ptr<Sensor>> sensors = create_sensors_for(sim0);
    //Get number of steps in scenario and start timer
    mono::precise_stopwatch stopwatch;

    /// initialize the vehicle, the first control command spawns the vehicle
    sim0.send_command(ApiMessage(123, EgoControl_ID, true, 
        {   {"forward_amount", 0.0}, 
            {"right_amount", 0.0},
            {"brake_amount", 0.0},
            {"drive_mode", 1}
        }));
    for(auto& sensor : sensors){
        sensor->StartSampleLoop();
    }

    sensors[0]->sample_callback = [](DataFrame* frame){
        auto& radarFrame = *static_cast<RadarFrame*>(frame);
        // std::cout << nlohmann::json(radarFrame.radarTargetListFrame->targets).dump() << std::endl;
    };

    std::cout << "Sampling sensor loop" << std::endl;
    int count = 0;
    while(true)
    {	
        sim0.sample_all(sensors);
    }
}

int main(int argc, char** argv)
{
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;
    
    //Read JSON files in cpp_client/config directory
    Configuration config(
        "cpp-client/parser_dev/simulator.json",
        "config/vehicle.json",
        "config/weather.json",
        "cpp-client/buffer_dev/scenario.json"
    );
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    state_test(sim0);
    
    return 0;
}
