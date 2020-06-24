// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#include <fstream>
#include <iostream>
#include <string>

#include "Configuration.h"


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
    std::cout << path << std::endl;
    try
    {
        std::ifstream in(path, std::ifstream::in);
        in >> j;
        in.close();
    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
	
	return j;
}
