// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#include "Configuration.h"

#include <fstream>
#include <iostream>
#include <string>


Configuration::Configuration(
    const std::string& simulatorConfigPath,
    const std::string& weatherConfigPath,
    const std::string& scenarioConfigPath
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
        scenario  = load(scenarioConfigPath);
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
