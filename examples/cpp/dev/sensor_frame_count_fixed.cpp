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

#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// #define IMG_WIDTH 4096
// #define IMG_HEIGHT 2160

#define IMG_WIDTH 1920
#define IMG_HEIGHT 1080

std::vector<std::shared_ptr<Sensor>> create_sensors_for(const Simulator& sim0)
{
    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;
    CameraConfig fc_config;
    fc_config.server_ip = sim0.getServerIp();
    fc_config.server_port = sim0.getServerPort();
    fc_config.listen_port = 8100;
    fc_config.location.z = 225;
    fc_config.rotation.pitch = -5;
    fc_config.resolution = Resolution(IMG_WIDTH,IMG_HEIGHT);
    fc_config.annotation.include_annotation = true;
    fc_config.annotation.desired_tags = {"traffic_sign"};
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(fc_config)));

    StateConfig s_config;
    s_config.server_ip = sim0.getServerIp();
    s_config.server_port = sim0.getServerPort();
    s_config.listen_port = 8101;
    s_config.desired_tags = {"ego"};
    s_config.include_obb = true;
    s_config.debug_drawing = false;
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<StateConfig>(s_config)));

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

    // Send configuraitons to the simulator
    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }
    return sensors;
}

void camera_test(Simulator& sim0){
    //Setup and Connect Sensors
    std::vector<std::shared_ptr<Sensor>> sensors = create_sensors_for(sim0);
    //Get number of steps in scenario and start timer
    mono::precise_stopwatch stopwatch;

    /// initialize the vehicle, the first control command spawns the vehicle
    nlohmann::json ego_msg;
    ego_msg["forward_amount"] =  0.0;
    ego_msg["right_amount"] = 0.0;
    ego_msg["brake_amount"] = 1.0;
    ego_msg["drive_mode"] = 1;
    sim0.send_command(ApiMessage(123, EgoControl_ID, true, ego_msg));

    for(auto& sensor : sensors){
        sensor->StartSampleLoop();
    }

    std::cout << "Sampling sensor loop" << std::endl;
    while(true)
    {	
        ClosedLoopStepCommandConfig stepCommand;
        stepCommand.time_step = 0.01f;
        sim0.send_command(ApiMessage(1234, ClosedLoopStepCommand_ID, true, stepCommand.dump()));
        sim0.sample_all(sensors);
        for(auto& sensor : sensors){
            std::cout << sensor->frame->sample_count << " ";
        }
        for(auto& sensor : sensors){
            std::cout << sensor->frame->game_time << " ";
        }
        for(auto& sensor : sensors){
            std::cout << sensor->frame->wall_time << " ";
        }
        std::cout << std::endl;
    }
}

int main(int argc, char** argv)
{
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;
    
    //Read JSON files in cpp_client/config directory
    Configuration config(
        "examples/cpp/buffer_dev/simulator_no_traffic_fixed_step.json",
        "config/weather.json",
        "config/scenario_config_multi_vehicle.json"
    );
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    camera_test(sim0);
    
    return 0;
}
