#pragma once
#include "json.hpp"

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;


class Configuration
{
public:
    // Configuration(int argc, char** argv);
    Configuration(
        const fs::path& simulatorConfigPath,
        const fs::path& vehicleConfigPath,
        const fs::path& weatherConfigPath,
        const fs::path& scenarioConfigPath
    );
    Configuration(
        const std::string& simulator_config_path = "config/simulator.json",
        const std::string& vehicle_config_path = "config/vehicle.json",
        const std::string& weather_config_path = "config/weather.json",
        const std::string& scenario_config_path = "config/scenario.json"
    );
    ~Configuration(){};
    nlohmann::json simulator;
    nlohmann::json vehicle;
    nlohmann::json weather;
    nlohmann::json scenario;
private:
    nlohmann::json load(const std::string& path);

};