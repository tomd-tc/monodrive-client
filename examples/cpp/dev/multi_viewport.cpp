#include <iostream>		  // std::cout
#include <chrono>		  // std::chrono
#include <thread>         // std::thread
#include <vector>

// monoDrive Includes
#include "Simulator.h"
#include "Configuration.h"  // holder for sensor, simulator, scenario, weather, and vehicle configurations
#include "Sensor.h"
#include "sensor_config.h"


int main(int argc, char** argv)
{
    std::string server_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port

    // read in JSON files for simulator configuration
    Configuration config(
        "examples/config/simulator_straightaway.json",
        "examples/config/weather.json",
        "examples/config/scenario_multi_vehicle_straightaway.json"
    );
    Simulator& sim = Simulator::getInstance(config, server_ip, server_port);

    if (!sim.configure()) {
        return -1;
    }

    // configure viewport cameras
    ViewportCameraConfig vp1;
    vp1.listen_port = 1;
    vp1.location.z = 200;
    vp1.rotation.yaw = -15;
    vp1.resolution = Resolution(1980, 1024);
    vp1.enable_hud = true;
    Sensor(std::make_unique<ViewportCameraConfig>(vp1)).configure();

    ViewportCameraConfig vp2;
    vp2.listen_port = 2;
    vp2.location.z = 200;
    vp2.rotation.yaw = 15;
    vp2.resolution = Resolution(1980, 1024);
    vp2.enable_hud = true;
    Sensor(std::make_unique<ViewportCameraConfig>(vp2)).configure();

    // send control command
    while (true)
    {
        sim.sendControl(0.1, 0.0, 0.0, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
