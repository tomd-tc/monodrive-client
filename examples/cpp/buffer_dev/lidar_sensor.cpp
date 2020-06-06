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

std::vector<std::shared_ptr<Sensor>> create_sensors_for(const Simulator& sim0)
{
    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;
    LidarConfig l_config;
    l_config.location.x = -10.f;
    l_config.location.z = 190.f;
    l_config.horizontal_resolution = 0.1f;
    l_config.n_lasers = 32;
    l_config.server_ip = sim0.getServerIp();
    l_config.server_port = sim0.getServerPort();
    l_config.listen_port = 8107;
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<LidarConfig>(l_config)));

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

void lidar_test(Simulator& sim0){
    //Setup and Connect Sensors
    std::vector<std::shared_ptr<Sensor>> sensors = create_sensors_for(sim0);

    /// initialize the vehicle, the first control command spawns the vehicle
    nlohmann::json ego_command;
    ego_command["forward_amount"] =  0.0;
    ego_command["right_amount"] =  0.0;
    ego_command["brake_amount"] =  0.0;
    ego_command["drive_mode"] =  1;
    sim0.send_command(ApiMessage(123, EgoControl_ID, true, ego_command));

    for(auto& sensor : sensors){
        sensor->StartSampleLoop();
    }

    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service);
    boost::asio::ip::udp::endpoint remote_endpoint;
    socket.open(boost::asio::ip::udp::v4());
    remote_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 2368);
    sensors[0]->sample_callback = [&remote_endpoint, &socket](DataFrame* frame){
        auto& lidarFrame = *static_cast<LidarFrame*>(frame);
        int count = 0;
        for(auto& packet : lidarFrame.packets){
            boost::system::error_code err;
            socket.send_to(boost::asio::buffer(&packet, sizeof(LidarPacket)), remote_endpoint, 0, err);
            std::this_thread::sleep_for(std::chrono::microseconds(1327));
        }
    };

    std::cout << "Sampling sensor loop" << std::endl;
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

    lidar_test(sim0);
    
    return 0;
}
