// Copyright 2017-2020 monoDrive, LLC. All Rights Reserved.
#pragma once

#include <string>
#include "config_types.h"
#include "JsonHelpers.h"
#include "SimulatorCommands.h"

class CommandBaseConfig
{
    public:
        CommandBaseConfig(){};
        virtual ~CommandBaseConfig(){};
        std::string type = "None";
        int reference =  0;
        bool success = true;
        friend void to_json(nlohmann::json& j,const CommandBaseConfig& config);
        friend void from_json(const nlohmann::json& j, CommandBaseConfig& config);
        virtual nlohmann::json dump(){
            throw std::runtime_error("CommandBaseConfig cannot be converted to JSON. Implement the conversion in the derived class.");
        }
        virtual ApiMessage message() {
            return ApiMessage(reference, type, success, dump());
        }
};

class EgoControlConfig : public CommandBaseConfig 
{
public:
    EgoControlConfig()
    {
        type = EgoControl_ID;
    }
    float forward_amount = 0.0;
    float right_amount = 0.0;
    float brake_amount = 0.0;
    int drive_mode = 1;
    virtual nlohmann::json dump(){
        return *this;
    }
};

class MapConfig : public CommandBaseConfig
{
public:
    MapConfig()
    {
        type = "GetMap";
    }
    std::string format = "geojson";
    std::string coordinates = "world";
    double point_delta{100.0};
    double orientation{-16.5};
    Location gis_anchor  = {
        40.410262f,
        -79.948172f,
        1.0f
    };
    virtual nlohmann::json dump(){
        return *this;
    }
};

class ClosedLoopStepCommandConfig : public CommandBaseConfig
{
public:
    ClosedLoopStepCommandConfig(){
        type = ClosedLoopStepCommand_ID;
    }
    float time_step = 0.01f;
    virtual nlohmann::json dump(){
        return *this;
    }
};

void inline to_json(nlohmann::json& j, const CommandBaseConfig& config)
{
    j = nlohmann::json{
        {"type", config.type},
    };
};
void inline from_json(const nlohmann::json& j, CommandBaseConfig& config)
{
    json_get(j, "type", config.type);
}
void inline to_json(nlohmann::json& j, const EgoControlConfig& config) {
    j = static_cast<CommandBaseConfig>(config);
    j["forward_amount"] = config.forward_amount;
    j["right_amount"] = config.right_amount;
    j["brake_amount"] = config.brake_amount;
    j["drive_mode"] = config.drive_mode;
}
void inline from_json(const nlohmann::json& j, EgoControlConfig& config) {
    json_get(j, "forward_amount", config.forward_amount);
    json_get(j, "right_amount", config.right_amount);
    json_get(j, "brake_amount", config.brake_amount);
    json_get(j, "drive_mode", config.drive_mode);
}
void inline to_json(nlohmann::json& j, const MapConfig& config) {
    j = static_cast<CommandBaseConfig>(config);
    j["format"] = config.format;
    j["coordinates"] = config.coordinates;
    j["point_delta"] = config.point_delta;
    j["orientation"] = config.orientation;
    j["gis_anchor"] = config.gis_anchor;
}
void inline from_json(const nlohmann::json& j, MapConfig& config) {
    json_get(j, "format", config.format);
    json_get(j, "coordinates", config.coordinates);
    json_get(j, "point_delta", config.point_delta);
    json_get(j, "orientation", config.orientation);
    json_get(j, "gis_anchor", config.gis_anchor);
}

void inline to_json(nlohmann::json& j, const ClosedLoopStepCommandConfig& config){
    j = static_cast<CommandBaseConfig>(config);
    j["time_step"] = config.time_step;
}

void inline from_json(const nlohmann::json& j, ClosedLoopStepCommandConfig& config) {
    json_get(j, "time_step", config.time_step);
}