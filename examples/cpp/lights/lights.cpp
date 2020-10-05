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
    config.simulator["map"] = "Straightaway5k_night";
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    // Configure simulator
    if (!sim0.configure()) {
        return -1;
    }

    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;
    StateConfig state_config;
    state_config.desired_tags = { "vehicle" };
    state_config.undesired_tags = { "static" };
    state_config.server_ip = sim0.getServerIp();
    state_config.server_port = sim0.getServerPort();
    state_config.listen_port = 8200;
    state_config.debug_drawing = true;
    state_config.undesired_tags = { "" };
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<StateConfig>(state_config)));

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

    sim0.sendCommand(ApiMessage(1002, VehicleLightsConfigCommand_ID, true, lights));

    std::cout << "Sampling sensor loop" << std::endl;
    int led_index = 0;
    int delta = 1;

    while (true)
    {
        sim0.sendControl(0.0, 0, 0, 1);
        sim0.sampleAll(sensors);

        for (int i = 0; i < lights.size();  i++) {
            auto& lightArray = lights[i];
            for (int j = 0; j < lightArray.lights.size(); j++) {
                int index = j + (i * lightArray.lights.size());

                if (index == led_index) {
                    float intensity = lightArray.lights[j].intensity > 0 ? 0 : 1500;
                    lightArray.lights[j].intensity = intensity;
                    lightArray.lights[j].backlight_intensity = intensity > 0 ? 400 : 0;
                }
                std::cout << index << ": " << lightArray.lights[j].intensity << std::endl;
            }
            std::cout << std::endl;
        }
        sim0.sendCommand(ApiMessage(1002, VehicleLightsConfigCommand_ID, true, lights));
        led_index += delta;
        if (led_index > ledCount)
            delta = -1;
        else if (led_index < 0)
            delta = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}
