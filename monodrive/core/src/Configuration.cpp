// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#include <fstream>
#include <iostream>
#include <string>

#include "Configuration.h"


Configuration::Configuration(
    const std::string& simulatorConfigPath,
    const std::string& weatherConfigPath,
    const std::string& scenarioConfigPath,
    const std::string& sensorsConfigPath
){
    if (!simulatorConfigPath.empty())
    {
        simulator = load(simulatorConfigPath);
    }
    if (!weatherConfigPath.empty())
    {
        weather = load(weatherConfigPath);
    }
    if (!scenarioConfigPath.empty())
    {
        scenario = load(scenarioConfigPath);
    }
    if (!sensorsConfigPath.empty())
    {
        sensorsConfig = load(sensorsConfigPath);
    }
}


nlohmann::json Configuration::load(const std::string& path)
{
    nlohmann::json j;
    try
    {
        std::ifstream in(path, std::ifstream::in);
        if(in.is_open()) {
            in >> j;
            in.close();
        } else {
            std::cerr << "WARNING! Unable to read configuration file at: " 
                << path << std::endl;
        }
    }
    catch(std::exception& e)
    {
        std::cerr << "WARNING! Exception thrown when parsing file: " << path << std::endl;
        std::cerr << e.what() << std::endl;
    }
	
	return j;
}



void Configuration::loadSensors(std::vector<std::shared_ptr<Sensor>>& sensors)
{
    if (!sensorsConfig.is_array())
    {
        std::cerr << "Cannot load sensors from invalid JSON array." << std::endl;
        return;
    }
    for (auto& s : sensorsConfig)
    {
        std::unique_ptr<SensorBaseConfig> cfg = sensorConfigFactory(s);
        if (!cfg)
        {
            std::cerr << "Could not load sensor config " << s.dump() << std::endl;
            continue;
        }
        sensors.push_back(std::make_shared<Sensor>(std::move(cfg)));
    }
}
