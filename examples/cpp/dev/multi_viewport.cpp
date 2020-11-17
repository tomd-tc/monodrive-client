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

    // configure primary viewport cameras
    ViewportCameraConfig vp0;
    vp0.server_ip = sim.getServerIp();
    vp0.server_port = sim.getServerPort();
    vp0.listen_port = 0;
    vp0.location.z = 400;
    vp0.location.x = -800;
    vp0.rotation.pitch = -15;
    vp0.enable_hud = true;
    Sensor(std::make_unique<ViewportCameraConfig>(vp0)).configure();

    // configure normal camera as viewports
    CameraConfig vp1;
    vp1.server_ip = sim.getServerIp();
    vp1.server_port = sim.getServerPort();
    vp1.listen_port = 1;
    vp1.enable_streaming = false;
    vp1.location.z = 150;
    vp1.location.y = -50;
    vp1.rotation.yaw = -60;
    vp1.resolution = Resolution(512, 512);
    vp1.viewport.enable_viewport =  true;
    vp1.viewport.window_offset = Resolution(0, 512);
    vp1.viewport.monitor_number = -1;
    Sensor(std::make_unique<CameraConfig>(vp1)).configure();

    CameraConfig vp2;
    vp2.server_ip = sim.getServerIp();
    vp2.server_port = sim.getServerPort();
    vp2.listen_port = 2;
    vp2.enable_streaming = false;
    vp2.location.z = 225;
    vp2.rotation.yaw = 0;
    vp2.resolution = Resolution(896, 512);
    vp2.viewport.enable_viewport =  true;
    vp2.viewport.window_offset = Resolution(512, 512);
    vp2.viewport.monitor_number = -1;
    Sensor(std::make_unique<CameraConfig>(vp2)).configure();

    CameraConfig vp3;
    vp3.server_ip = sim.getServerIp();
    vp3.server_port = sim.getServerPort();
    vp3.listen_port = 3;
    vp3.enable_streaming = false;
    vp3.location.z = 150;
    vp3.location.y = 50;
    vp3.rotation.yaw = 60;
    vp3.resolution = Resolution(512, 512);
    vp3.viewport.enable_viewport = true;
    vp3.viewport.window_offset = Resolution(1408, 512);
    vp3.viewport.monitor_number = -1;
    Sensor(std::make_unique<CameraConfig>(vp3)).configure();

    // configure 360 camera as viewport
    Camera360Config vp4;
    vp4.server_ip = sim.getServerIp();
    vp4.server_port = sim.getServerPort();
    vp4.listen_port = 4;
    vp4.enable_streaming = false;
    vp4.location.z = 225;
    vp4.resolution = Resolution(1024, 512);
    vp4.face_size = 512;
    vp4.viewport.enable_viewport = true;
    vp4.viewport.window_offset = Resolution(448, 0);
    vp4.viewport.monitor_number = -1;
    Sensor(std::make_unique<Camera360Config>(vp4)).configure();

    // send control command
    while (true)
    {
        sim.sendControl(0.1, 0.0, 0.0, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
