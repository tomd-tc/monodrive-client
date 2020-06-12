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


int main(int argc, char** argv)
{
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;

    //Read JSON files in cpp_client/config directory
    Configuration config(
        "examples/config/simulator_no_traffic.json",
        "examples/config/weather.json",
        "examples/config/scenario_multi_vehicle_almono.json"
    );

    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;
    SemanticLidarConfig sl_config;
    sl_config.location.x = -10.f;
    sl_config.location.z = 190.f;
    sl_config.horizontal_resolution = 0.4f;
    sl_config.n_lasers = 16;
    sl_config.server_ip = sim0.getServerIp();
    sl_config.server_port = sim0.getServerPort();
    sl_config.listen_port = 8107;
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<SemanticLidarConfig>(sl_config)));

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

    /// initialize the vehicle, the first control command spawns the vehicle
    sim0.sendControl(0.1, 0.0, 0.0, 1);

    // define callback to forward lidar data to veloview
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service);
    boost::asio::ip::udp::endpoint remote_endpoint;
    socket.open(boost::asio::ip::udp::v4());
    remote_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 2368);
    sensors[0]->sampleCallback = [&remote_endpoint, &socket](DataFrame* frame){
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
        sim0.sampleAll(sensors);
    }

    return 0;
}
