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

    StateConfig s_config;
    s_config.server_ip = sim0.getServerIp();
    s_config.server_port = sim0.getServerPort();
    s_config.listen_port = 8101;
    s_config.desired_tags = {"vehicle", "traffic_sign"};
    s_config.include_obb = true;
    s_config.debug_drawing = true;
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<StateConfig>(s_config)));

    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim0.getServerIp();
    vp_config.server_port = sim0.getServerPort();
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256,256);
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
        auto& stateFrame = *static_cast<StateFrame*>(frame);
        // std::cout << stateFrame.vehicles.size() << std::endl;
        // std::cout << nlohmann::json(stateFrame.vehicles).dump(1) << std::endl;
        // std::cout << nlohmann::json(stateFrame.objects).dump(1) << std::endl;
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
        "config/weather.json",
        "config/scenario_config_single_vehicle.json"
    );
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    state_test(sim0);
    
    return 0;
}
