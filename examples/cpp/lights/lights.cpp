//monoDrive Includes
#include "Simulator.h"
#include "Configuration.h"
#include "Sensor.h"
#include "sensor_config.h"
#include "command_config.h"
#include "Stopwatch.h"


int main(int argc, char** argv)
{
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;

    //Read JSON files in cpp_client/config directory
    Configuration config(
        "examples/config/simulator_straightaway_night.json",
        "examples/config/weather.json",
        "examples/config/scenario_headlights.json"
    );
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    // Configure simulator
    if (!sim0.configure()) {
        return -1;
    }

    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim0.getServerIp();
    vp_config.server_port = sim0.getServerPort();
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256, 256);
    vp_config.enable_hud = true;
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    // create headlights configuration
    std::vector<LEDArrayConfig> lights;
    LEDArrayConfig lf_light_config;
    lf_light_config.array_id = "LF";
    lf_light_config.location = Location(171, -54.5, 81);
    int ledCount = 0;
    int yaw = 6;
    for (int i = 0; i < 8; i++) {
        LEDConfig led_config;
        led_config.led = i;
        led_config.location = Location(0, i * -2, 0);
        led_config.rotation = Rotation(yaw, -1, 0);
        led_config.intensity = 1500;
        led_config.backlight_intensity = 400;
        led_config.inner_cone_angle = 2;
        led_config.outer_cone_angle = 6;
        led_config.attenuation_radius = 5000;
        led_config.temperature = 8000;
        led_config.backlight_temperature = 8000;
        //led_config.ies_profile = "Materials/IES/parallel-beam.parallel-beam";
        lf_light_config.lights.push_back(led_config);
        ledCount++;
        yaw -= 3;
    }
    lights.push_back(lf_light_config);

    LEDArrayConfig rf_light_config;
    rf_light_config.array_id = "RF";
    rf_light_config.location = Location(171, 54.5, 81);
    yaw = -6;
    for (int i = 0; i < 8; i++) {
        LEDConfig led_config;
        led_config.led = i;
        led_config.location = Location(0, i * 2, 0);
        led_config.rotation = Rotation(yaw, -1, 0);
        led_config.intensity = 1500;
        led_config.backlight_intensity = 400;
        led_config.inner_cone_angle = 2;
        led_config.outer_cone_angle = 6;
        led_config.attenuation_radius = 5000;
        led_config.temperature = 8000;
        led_config.backlight_temperature = 8000;
//        led_config.ies_profile = "Materials/IES/parallel-beam.parallel-beam";
        rf_light_config.lights.push_back(led_config);
        ledCount++;
        yaw += 3;
    }
    lights.push_back(rf_light_config);

    LightsConfig lightsConfig;
    lightsConfig.actor_id = sim0.getEgoVehicleId();
    lightsConfig.lights = lights;

    // send lights config command
    sim0.sendCommand(ApiMessage(1002, VehicleLightsConfigCommand_ID, true, lightsConfig));

    std::cout << "Sampling sensor loop" << std::endl;
    int led_index = 0;
    int delta = 1;
    uint64_t iterations = 0;
    uint64_t total_time = 0;
    while (true)
    {
        sim0.sendControl(0.0, 0, 0, 1);

        nlohmann::json update_config({
                { "actor_id", lightsConfig.actor_id },
                { "lights", nlohmann::json::array() }
            });
        for (int i = 0; i < lightsConfig.lights.size();  i++) {
            auto& lightArray = lightsConfig.lights[i];
            update_config["lights"].push_back({
                    { "array_id", lightArray.array_id },
                    { "lights", nlohmann::json::array() }
                });
            for (int j = 0; j < lightArray.lights.size(); j++) {
                int index = j + (i * lightArray.lights.size());

                if (index == led_index) {
                    float intensity = lightArray.lights[j].intensity > 0 ? 0 : 1500;
                    lightArray.lights[j].intensity = intensity;
                    lightArray.lights[j].backlight_intensity = intensity > 0 ? 400 : 0;
                    update_config["lights"].back()["lights"].push_back({
                            { "led", j },
                            { "intensity", lightArray.lights[j].intensity },
                            { "backlight_intensity", lightArray.lights[j].backlight_intensity }
                        });
                }
            }
        }
        
        {
            mono::precise_stopwatch stopwatch;
            sim0.sendCommand(ApiMessage(1002, VehicleLightsUpdateCommand_ID, true, update_config));
            total_time += stopwatch.elapsed_time<uint64_t, std::chrono::milliseconds>();
            iterations++;
        }

        led_index += delta;
        if (led_index > ledCount)
            delta = -1;
        else if (led_index < 0)
            delta = 1;

        if (iterations % 100 == 0 && iterations > 0) {
            std::cout << (total_time / (double)iterations) << " ms" << std::endl;
        }
    }

    return 0;
}
