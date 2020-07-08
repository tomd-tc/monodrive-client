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
    ViewportCameraConfig vp0;
    vp0.listen_port = 0;
    vp0.location.z = 400;
    vp0.location.x = -800;
    vp0.rotation.pitch = -15;
    vp0.enable_hud = true;
    Sensor(std::make_unique<ViewportCameraConfig>(vp0)).configure();

    ViewportCameraConfig vp1;
    vp1.listen_port = 1;
    vp1.location.z = 125;
    vp1.location.y = -25;
    vp1.rotation.yaw = -60;
    vp1.resolution = Resolution(512, 512);
    vp1.window_offset = Resolution(0, 256);
    vp1.monitor_number = 0;
    Sensor(std::make_unique<ViewportCameraConfig>(vp1)).configure();

    ViewportCameraConfig vp2;
    vp2.listen_port = 2;
    vp2.location.z = 125;
    vp2.rotation.yaw = 0;
    vp2.resolution = Resolution(896, 512);
    vp2.window_offset = Resolution(512, 256);
    vp2.monitor_number = 0;
    Sensor(std::make_unique<ViewportCameraConfig>(vp2)).configure();

    ViewportCameraConfig vp3;
    vp3.listen_port = 3;
    vp3.location.z = 125;
    vp3.location.y = 25;
    vp3.rotation.yaw = 60;
    vp3.resolution = Resolution(512, 512);
    vp3.window_offset = Resolution(1408, 256);
    vp3.monitor_number = 0;
    Sensor(std::make_unique<ViewportCameraConfig>(vp3)).configure();

    // send control command
    while (true)
    {
        sim.sendControl(0.1, 0.0, 0.0, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
