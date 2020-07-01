// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#include "Jobs.h"
#include <fstream>

#define GCC_VERSION __GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__
#if GCC_VERSION >= 40900
#include "cxxopts.hpp"  // requires regex
#endif


Job::Job(int argc, char** argv)
{
    parseArguments(argc, argv);
}

bool Job::setResult(const Result& result)
{
    std::ofstream file(resultsPath);
	if (!file.is_open())
	{
        std::cerr << "Unable to write to file: " << std::endl;
        return false;
	}
	file << result.dump().dump(2);
    return true;
}

void Job::parseArguments(int argc, char** argv)
{
#if GCC_VERSION >= 40900
    cxxopts::Options options("monoDrive Simulator Jobs interface");
    options.allow_unrecognised_options();
    options.add_options()
     (AssetDir_FLAG, "", cxxopts::value<std::string>())
     (Simulator_FLAG, "", cxxopts::value<std::string>())
     (Scenario_FLAG, "", cxxopts::value<std::string>())
     (Weather_FLAG, "", cxxopts::value<std::string>())
     (Vehicle_FLAG, "", cxxopts::value<std::string>())
     (Sensors_FLAG, "", cxxopts::value<std::string>())
     (Results_FLAG, "", cxxopts::value<std::string>())
    ;
    auto cla = options.parse(argc, argv);

    if (cla.count(AssetDir_FLAG))
    {
        assetDirPath = cla[AssetDir_FLAG].as<std::string>();
    }
    if (cla.count(Simulator_FLAG))
    {
        simulatorPath = cla[Simulator_FLAG].as<std::string>();
    }
    if (cla.count(Scenario_FLAG))
    {
        scenarioPath = cla[Scenario_FLAG].as<std::string>();
    }
    if (cla.count(Weather_FLAG))
    {
        weatherPath = cla[Weather_FLAG].as<std::string>();
    }
    if (cla.count(Vehicle_FLAG))
    {
        vehiclePath = cla[Vehicle_FLAG].as<std::string>();
    }
    if (cla.count(Sensors_FLAG))
    {
        sensorsPath = cla[Sensors_FLAG].as<std::string>();
    }
    if (cla.count(Results_FLAG))
    {
        resultsPath = cla[Results_FLAG].as<std::string>();
    }
#else
    std::cout << "Warning unable to parse command line args with gcc < 4.9" << std::endl;
#endif
}

nlohmann::json ResultMetric::dump() const
{
    return *this;
}

nlohmann::json Result::dump() const
{
    return *this;
}

void to_json(nlohmann::json& j, const ResultMetric& m)
{
    j["name"] = m.name;
    j["score"] = m.score;
}
void from_json(const nlohmann::json& j, ResultMetric& m)
{
    json_get(j, "name", m.name);
    json_get(j, "score", m.score);
}

void to_json(nlohmann::json& j, const Result& r)
{
    j["pass"] = r.pass;
    j["message"] = r.message;
    j["metrics"] = r.metrics;
}
void from_json(const nlohmann::json& j, Result& r)
{
    json_get(j, "pass", r.pass);
    json_get(j, "message", r.message);
    json_get(j, "metrics", r.metrics);
}
