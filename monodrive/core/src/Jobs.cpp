// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#include "Jobs.h"

#include <chrono>
#include <thread>
#include <fstream>
#include <boost/filesystem.hpp>

#define GCC_VERSION __GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__
#if GCC_VERSION >= 40900 || _MSC_VER >= 1900
#include "cxxopts.hpp"  // requires regex
#endif

namespace fs = boost::filesystem;


Job::Job(int argc, char** argv) : argc(argc), argv(argv)
{
    parseArguments(argc, argv);
}


int Job::run(std::function<int (int, char**, Job*)> main)
{
    if (!loop)
    {
        return main(argc, argv, this);
    }

    while (true)
    {
        // wait until ready
        while (true)
        {
            std::this_thread::sleep_for(std::chrono::seconds(POLL_INTERVAL));
            JobState state = getState();
            std::cout << "Job status: " << JOB_STATE_NAMES.at(state) << std::endl;
            if (state == JobState::READY)
            {
                break;
            }
        }

        // run uut
        int res = main(argc, argv, this);

        // update job status
        JobState state = res ? JobState::FAILED : JobState::COMPLETED;
        setState(state);
        std::cout << "Job status: " << JOB_STATE_NAMES.at(state) << std::endl;
    }
    return 0;
}

bool Job::setState(JobState state)
{
    if (assetDirPath.empty())
    {
        std::cerr << "ERROR! No assets directory provided to locate state file" << std::endl;
        return false;
    }
    std::string path = (fs::path(assetDirPath) / fs::path(STATE_FILE)).string();
    std::ofstream file(path);
	if (!file.is_open())
	{
        std::cerr << "ERROR! Unable to write to state file: " << path << std::endl;
        return false;
	}
	file << JOB_STATE_NAMES.at(state);
    return true;
}


bool Job::setResult(const Result& result)
{
    std::string path;
    if (!assetDirPath.empty())
    {
        path = (fs::path(assetDirPath) / fs::path(RESULTS_FILE)).string();
    }
    if (!resultsPath.empty())
    {
        path = resultsPath;
    }
    std::ofstream file(path);
	if (!file.is_open())
	{
        std::cerr << "ERROR! Unable to write to results file: " << path << std::endl;
        return false;
	}
	file << result.dump().dump(2);
    return true;
}


JobState Job::getState()
{
    if (assetDirPath.empty())
    {
        std::cerr << "ERROR! No assets directory provided to locate state file" << std::endl;
        return JobState::UNDEFINED;
    }
    std::string path = (fs::path(assetDirPath) / fs::path(STATE_FILE)).string();
    std::ifstream file(path);
    if (!file.is_open())
	{
        std::cout << "WARNING! Unable to read state file: " << path << std::endl;
        return JobState::UNDEFINED;
	}
    std::string state;
    std::getline(file, state);

    for (auto const& s : JOB_STATE_NAMES)
    {
        if (s.second == state)
        {
            return s.first;
        }
    }
    return JobState::UNDEFINED;
}


Configuration Job::getConfig()
{
    std::string sim;
    std::string scenario;
    std::string weather;
    std::string sensors;

    // set paths from assets directory if provided
    if (!assetDirPath.empty())
    {
        sim = (fs::path(assetDirPath) / fs::path(SIMULATOR_FILE)).string();
        scenario = (fs::path(assetDirPath) / fs::path(SCENARIO_FILE)).string();
        weather = (fs::path(assetDirPath) / fs::path(WEATHER_FILE)).string();
        sensors = (fs::path(assetDirPath) / fs::path(SENSORS_FILE)).string();
    }

    // set/update with individual flags
    if (!simulatorPath.empty())
    {
        sim = simulatorPath;
    }
    if (!scenarioPath.empty())
    {
        scenario = scenarioPath;
    }
    if (!weatherPath.empty())
    {
        weather = weatherPath;
    }
    if (!sensorsPath.empty())
    {
        sensors = sensorsPath;
    }

    // create config
    return Configuration(sim, weather, scenario, sensors);
}


void Job::parseArguments(int argc, char** argv)
{
#if GCC_VERSION >= 40900 || _MSC_VER >= 1900
    cxxopts::Options options("monoDrive Simulator Jobs interface");
    options.allow_unrecognised_options();
    options.add_options()
     (ASSET_DIR_FLAG, "", cxxopts::value<std::string>())
     (SIMULATOR_FLAG, "", cxxopts::value<std::string>())
     (SCENARIO_FLAG, "", cxxopts::value<std::string>())
     (WEATHER_FLAG, "", cxxopts::value<std::string>())
     (VEHICLE_FLAG, "", cxxopts::value<std::string>())
     (SENSORS_FLAG, "", cxxopts::value<std::string>())
     (RESULTS_FLAG, "", cxxopts::value<std::string>())
     (LOOP_FLAG, "")
     (HELP_FLAG, "")
    ;
    auto cla = options.parse(argc, argv);

    if (cla.count(HELP_FLAG))
    {
        std::cout << options.help() << std::endl;
        exit(0);
    }

    if (cla.count(ASSET_DIR_FLAG))
    {
        assetDirPath = cla[ASSET_DIR_FLAG].as<std::string>();
    }
    if (cla.count(SIMULATOR_FLAG))
    {
        simulatorPath = cla[SIMULATOR_FLAG].as<std::string>();
    }
    if (cla.count(SCENARIO_FLAG))
    {
        scenarioPath = cla[SCENARIO_FLAG].as<std::string>();
    }
    if (cla.count(WEATHER_FLAG))
    {
        weatherPath = cla[WEATHER_FLAG].as<std::string>();
    }
    if (cla.count(VEHICLE_FLAG))
    {
        vehiclePath = cla[VEHICLE_FLAG].as<std::string>();
    }
    if (cla.count(SENSORS_FLAG))
    {
        sensorsPath = cla[SENSORS_FLAG].as<std::string>();
    }
    if (cla.count(RESULTS_FLAG))
    {
        resultsPath = cla[RESULTS_FLAG].as<std::string>();
    }
    if (cla.count(LOOP_FLAG))
    {
        loop = true;
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
