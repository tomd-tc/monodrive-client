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
    LEDArrayConfig lf_light_config;
    lf_light_config.array_id = "LF";
    lf_light_config.location = Location(98, -48, 75);
    for (int i = 0; i < 3; i++) {
        LEDConfig led_config;
        led_config.led = i;
        led_config.location = Location(0, i * -10, 0);
        led_config.intensity = 1000;
        led_config.inner_cone_angle = 5;
        led_config.outer_cone_angle = 10;
        led_config.attenuation_radius = 2000;
        led_config.temperature = 8000;
        led_config.color.a = 255;
        led_config.color.r = 255;
        led_config.color.g = 100;
        led_config.color.b = 100;
        lf_light_config.lights.push_back(led_config);
    }
    lights.push_back(lf_light_config);

    LEDArrayConfig rf_light_config;
    rf_light_config.array_id = "RF";
    rf_light_config.location = Location(98, 48, 75);
    for (int i = 0; i < 3; i++) {
        LEDConfig led_config;
        led_config.led = i;
        led_config.location = Location(0, i * 10, 0);
        led_config.intensity = 1000;
        led_config.inner_cone_angle = 5;
        led_config.outer_cone_angle = 10;
        led_config.attenuation_radius = 2000;
        led_config.temperature = 8000;
        led_config.color.a = 255;
        led_config.color.r = 100;
        led_config.color.g = 100;
        led_config.color.b = 255;
        rf_light_config.lights.push_back(led_config);
    }
    lights.push_back(rf_light_config);

    sim0.sendCommand(ApiMessage(1002, VehicleLightsConfigCommand_ID, true, lights));

    std::cout << nlohmann::json(lights) << std::endl;

    std::cout << "Sampling sensor loop" << std::endl;
    int led_index = 0;
    while (true)
    {
        sim0.sendControl(0.0, 0, 0, 1);
        sim0.sampleAll(sensors);

        for (int i = 0; i < lights.size();  i++) {
            auto& lightArray = lights[i];
            for (int j = 0; j < lightArray.lights.size(); j++) {
                int index = j + (i * (lights.size() + 1));
                float intensity = /*(index >= led_index - 1 && index <= led_index + 1)*/index == led_index ? 0 : 1000;
                lightArray.lights[j].intensity = intensity;
                std::cout << index << ": " << intensity << std::endl;
            }
            std::cout << std::endl;
        }
        sim0.sendCommand(ApiMessage(1002, VehicleLightsConfigCommand_ID, true, lights));
        led_index++;
        if (led_index > 6)
            led_index = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
