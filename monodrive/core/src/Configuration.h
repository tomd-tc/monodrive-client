#pragma once
#include "json.hpp"


class Configuration
{
public:
    // Configuration(int argc, char** argv);
    Configuration() {};
    Configuration(
        const std::string& simulatorConfigPath,
        const std::string& weatherConfigPath,
        const std::string& scenarioConfigPath
    );
    ~Configuration(){};
    nlohmann::json simulator;
    nlohmann::json weather;
    nlohmann::json scenario;
private:
    nlohmann::json load(const std::string& path);

};
