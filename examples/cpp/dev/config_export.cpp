#include <iostream>
#include <fstream>

//monoDrive Includes
#include "Configuration.h"
#include "sensor_config.h"

#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

// Simple tool take all configuration classes and dump them to JSON files
int main(int argc, char** argv)
{
    Configuration config(
        "examples/config/simulator.json",
        "examples/config/weather.json",
        "examples/config/scenario.json"
    );

    // NOTE: When a new sensor is added, just put the config in this vector and 
    // it will be included in the output directory
    std::vector<SensorBaseConfig*> configs {
        new CameraConfig(),
        new CollisionConfig(),
        new DepthCameraConfig(),
        new GPSConfig(),
        new FisheyeCameraConfig(),
        new IMUConfig(),
        new LidarConfig(),
        new OccupancyGridConfig(),
        new RadarConfig(),
        new RPMConfig(),
        new SemanticCameraConfig(),
        new SemanticLidarConfig(),
        new SensorBaseConfig(),
        new StateConfig(),
        new UltrasonicConfig(),
        new ViewportCameraConfig()
    };

    // Dump all the simulator configuration files
    fs::path output_dir = "config_export";
    fs::create_directories(output_dir);
    std::ofstream out_file(output_dir / "simulator.json");
    out_file << config.simulator.dump(4) << std::endl;
    out_file.close();
    out_file.open(output_dir / "weather.json");
    out_file << config.weather.dump(4) << std::endl;
    out_file.close();
    out_file.open(output_dir / "scenario.json");
    out_file << config.scenario.dump(4) << std::endl;
    out_file.close();
    
    // Dump all the sensor configuration files
    for(auto& config : configs) {
        out_file.open(output_dir / (config->type + ".json"));
        out_file << config->dump().dump(4) << std::endl;
        out_file.close();
    }

    return 0;
}
