//monoDrive Includes
#include "Simulator.h"
#include "Configuration.h"
#include "Sensor.h"
#include "sensor_config.h"
#include "command_config.h"

int main(int argc, char** argv)
{
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;

    //Read JSON files in cpp_client/config directory
    Configuration config(
        "examples/config/simulator_straightaway.json",
        "examples/config/weather.json",
        "examples/config/scenario_multi_vehicle_straightaway.json"
    );
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    // Configure simulator
    if(!sim0.configure()){
        return -1;
    }

    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;

    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim0.getServerIp();
    vp_config.server_port = sim0.getServerPort();
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256,256);
    vp_config.enable_hud = true;
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    // Send configurations to the simulator
    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }

    // Setup the autopilot controlled vehicles
    AutopilotControlConfig apc_config;
    apc_config.set_speed = 100.0; // Desired speed of vehicle in mph
    apc_config.negotiated_speed = 100.0; // Negotiated speed of vehicle in mph
    apc_config.headway = 1000.0; // Amount of headway to allow between vehicles in centimeters
    apc_config.lane_change = 2; // Request to change lanes (2 - both ways)
    apc_config.autopilot_engaged = true; // Autopilot control is allowed
    apc_config.gear = "D1"; // Gear indicator
    apc_config.left_lane_change = false; // Left lane change indicator
    apc_config.right_lane_change = true; // Right lane change indicator
    apc_config.manual_override = false; // Manual override indicator
    sim0.sendCommand(apc_config.message());

    std::cout << "Sampling sensor loop" << std::endl;
    while(true)
    {
        sim0.sendControl(0.3, 0, 0, 1);
        sim0.sampleAll(sensors);
    }

    return 0;
}
