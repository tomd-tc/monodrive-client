// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once

#include "json.hpp"
#include "JsonHelpers.h"
#include "Configuration.h"

// flags
#define ASSET_DIR_FLAG "md_assets"
#define SIMULATOR_FLAG "md_simulator"
#define SCENARIO_FLAG "md_scenario"
#define WEATHER_FLAG "md_weather"
#define VEHICLE_FLAG "md_vehicle"
#define SENSORS_FLAG "md_sensors"
#define RESULTS_FLAG "md_results"
#define LOOP_FLAG "md_loop"
#define HEKO_FLAG "md_help"

// paths
#define ASSET_DIR "./mdassets"
#define STATE_FILE "STATUS"
#define SIMULATOR_FILE "simulator.json"
#define SCENARIO_FILE "scenario.json"
#define WEATHER_FILE "weather.json"
#define VEHICLE_FILE "vehicle.json"
#define SENSORS_FILE "sensors.json"
#define RESULTS_FILE "results.json"
#define REPORT_FILE "results_full.json"

// job state polling interval
#define POLL_INTERVAL 3


// enum for allowable job states during processing
enum JobState
{
    ASSIGNED = 0,
    CONFIGURING = 1,
    READY = 2,
    RUNNING = 3,
    FINISHING = 4,
    COMPLETED = 5,
    FAILED = 6
};
static std::map<JobState, std::string> const JOB_STATE_NAMES = {
    {JobState::ASSIGNED, "ASSIGNED"},
    {JobState::CONFIGURING, "CONFIGURING"},
    {JobState::READY, "READY"},
    {JobState::FINISHING, "FINISHING"},
    {JobState::COMPLETED, "COMPLETED"},
    {JobState::FAILED, "FAILED"}
};


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

    int run(std::function<int (int, char**, Job*)> main);
    bool setState(JobState state);
    bool setResult(const Result& result);
    JobState getState();
    Configuration getConfig();

private:
    void parseArguments(int argc, char** argv);

    std::string assetDirPath;
    std::string simulatorPath;
    std::string scenarioPath;
    std::string weatherPath;
    std::string vehiclePath;
    std::string sensorsPath;
    std::string resultsPath;
    bool loop;

    int argc;
    char** argv;
};
