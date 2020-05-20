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

std::vector<std::shared_ptr<Sensor>> create_sensors_for(const Simulator& sim0)
{
    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;
    // for(int i = 0; i < 5; ++i){
        RadarConfig r_config;
        r_config.location.x = 300.f;
        r_config.location.z = 50.f;
        r_config.paint_targets = false;
        r_config.send_radar_cube = true;
        r_config.max_radar_returns = 4500;
        r_config.sbr.ray_division_y = 10.f;
        r_config.sbr.ray_division_z = 10.f;
        // r_config.sbr.debug_scan = true;
        r_config.sbr.debug_rescan = true;
        r_config.sbr.debug_frustum = false;
        r_config.server_ip = sim0.getServerIp();
        r_config.server_port = sim0.getServerPort();
        // r_config.listen_port = 8102 + i;
        r_config.listen_port = 8102;
        sensors.push_back(std::make_shared<Sensor>(std::make_unique<RadarConfig>(r_config)));
    // }

    // RadarConfig r2;
    // r2.location.x = 300.f;
    // r2.location.z = 50.f;
    // r2.paint_targets = false;
    // r2.send_radar_cube = true;
    // r2.max_radar_returns = 100;
    // r2.sbr.ray_division_y = 20.f;
    // r2.sbr.ray_division_z = 20.f;
    // // r_config.sbr.debug_scan = true;
    // r2.sbr.debug_rescan = true;
    // r2.sbr.debug_frustum = false;
    // r2.server_ip = sim0.getServerIp();
    // r2.server_port = sim0.getServerPort();
    // r2.listen_port = 8103;
    // sensors.push_back(std::make_shared<Sensor>(std::make_unique<RadarConfig>(r2)));

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
    nlohmann::json ego_command;
    ego_command["forward_amount"] =  0.1;
    ego_command["right_amount"] =  0.0;
    ego_command["brake_amount"] =  0.0;
    ego_command["drive_mode"] =  1;
    sim0.send_command(ApiMessage(123, EgoControl_ID, true, ego_command));

    for(auto& sensor : sensors){
        sensor->StartSampleLoop();
    }

    sensors[0]->sample_callback = [](DataFrame* frame){
        auto& radarFrame = *static_cast<RadarFrame*>(frame);
        std::cout << nlohmann::json(radarFrame.radarTargetListFrame->targets).dump() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
        "config/simulator_no_traffic.json",
        "config/vehicle.json",
        "config/weather.json",
        "config/scenario_config_single_vehicle.json"
    );
    // config.simulator["map"] = "RadarCube";
    // config.scenario["frame"][0]["position"] = {0.f, 0.f, 20.f};
    // config.scenario["frame"][0]["velocity"] = {0.f, 0.f, 0.f};

    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    state_test(sim0);
    
    return 0;
}
