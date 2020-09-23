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
    if (!sim0.configure()) {
        return -1;
    }

    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;

    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim0.getServerIp();
    vp_config.server_port = sim0.getServerPort();
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256, 256);
    vp_config.enable_hud = true;
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    // Send configurations to the simulator
    std::cout << "***********ALL SENSOR's CONFIGS*******" << std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }

    std::vector<LEDArrayConfig> lights;
    LEDArrayConfig light_config;
    light_config.array_id = "LF";
    light_config.location = Location(98, -48, 75);
    for (int i = 0; i < 3; i++) {
        LEDConfig led_config;
        led_config.location = Location(0, i * -4, 0);
        led_config.intensity = 50;
        light_config.lights.push_back(led_config);
    }
    lights.push_back(light_config);

    light_config.array_id = "RF";
    light_config.location = Location(98, 48, 75);
    for (int i = 0; i < 3; i++) {
        LEDConfig led_config;
        led_config.location = Location(0, i * 4, 0);
        led_config.intensity = 10;
        light_config.lights.push_back(led_config);
    }
    lights.push_back(light_config);

    sim0.sendCommand(ApiMessage(1002, VehicleLightsConfigCommand_ID, true, lights));

    std::cout << "Sampling sensor loop" << std::endl;
    while (true)
    {
        sim0.sendControl(0.0, 0, 0, 1);
        sim0.sampleAll(sensors);
    }

    return 0;
}
