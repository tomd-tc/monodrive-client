// Copyright 2017-2020 monoDrive, LLC. All Rights Reserved.
#pragma once

#include "json.hpp"
#include "cxxopts.hpp"
#include "JsonHelpers.h"

#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

// flags
#define AssetDir_FLAG "md_assets"
#define Simulator_FLAG "md_simulator"
#define Scenario_FLAG "md_scenario"
#define Weather_FLAG "md_weather"
#define Vehicle_FLAG "md_vehicle"
#define Sensors_FLAG "md_sensors"
#define Results_FLAG "md_results"


// result metric data model
class ResultMetric
{
public:
    std::string name = "";
    float score = 0.0;
    virtual nlohmann::json dump() const;
};


// result data model
class Result
{
public:
    bool pass = false;
    std::string message = "";
    std::vector<ResultMetric> metrics;
    virtual nlohmann::json dump() const;
};


// result serialization
void to_json(nlohmann::json& j, const ResultMetric& r);
void from_json(const nlohmann::json& j, ResultMetric& r);
void to_json(nlohmann::json& j, const Result& r);
void from_json(const nlohmann::json& j, Result& r);


// job class for batch processing
class Job
{
public:
    Job(int argc, char** argv);
    ~Job(){};

    bool setResult(const Result& result);

private:
    void parseArguments(int argc, char** argv);

    fs::path assetDirPath;
    fs::path simulatorPath;
    fs::path scenarioPath;
    fs::path weatherPath;
    fs::path vehiclePath;
    fs::path sensorsPath;
    fs::path resultsPath;
};
