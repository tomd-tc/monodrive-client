#pragma once
#include "json.hpp"

#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;


class Configuration
{
public:
    // Configuration(int argc, char** argv);
    Configuration(
        const fs::path& simulatorConfigPath = {""},
        const fs::path& weatherConfigPath = {""},
        const fs::path& scenarioConfigPath = {""}
    );
    ~Configuration(){};
    nlohmann::json simulator;
    nlohmann::json weather;
    nlohmann::json scenario;
private:
    nlohmann::json load(const fs::path& path);

};
