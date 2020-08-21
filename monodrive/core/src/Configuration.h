// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once
#include "json.hpp"
#include "sensor_config.h"
#include "Sensor.h"


class Configuration
{
public:
    // Configuration(int argc, char** argv);
    Configuration() {};
    Configuration(const std::string& simulatorConfigPath)
    : Configuration(simulatorConfigPath, "", "", "") {};
    Configuration(
        const std::string& simulatorConfigPath,
        const std::string& weatherConfigPath
    ) : Configuration(simulatorConfigPath, weatherConfigPath, "", "") {};
    Configuration(
        const std::string& simulatorConfigPath,
        const std::string& weatherConfigPath,
        const std::string& scenarioConfigPath
    ) : Configuration(simulatorConfigPath, weatherConfigPath, scenarioConfigPath, "") {};
    Configuration(
        const std::string& simulatorConfigPath,
        const std::string& weatherConfigPath,
        const std::string& scenarioConfigPath,
        const std::string& sensorsConfigPath
    );
    ~Configuration(){};
    nlohmann::json simulator;
    nlohmann::json weather;
    nlohmann::json scenario;
    nlohmann::json sensorsConfig;

    bool loadSensors(std::vector<std::shared_ptr<Sensor>>& sensors);

   private:
    nlohmann::json load(const std::string& path);

};
