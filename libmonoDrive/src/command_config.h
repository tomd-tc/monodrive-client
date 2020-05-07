#pragma once

#include <string>
#include "config_types.h"
#include "JsonHelpers.h"

class CommandBaseConfig
{
    public:
        CommandBaseConfig(){};
        virtual ~CommandBaseConfig(){};
        std::string server_ip = "127.0.0.1";
        int server_port = 8999;
        std::string type = "None";
        int listen_port = 0;
        friend void to_json(nlohmann::json& j,const CommandBaseConfig& config);
        friend void from_json(const nlohmann::json& j, CommandBaseConfig& config);
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
    std::string format = "point_array";
    std::string coordinates = "world";
    double point_delta{100.0};
    double orientation{-16.5};
    Location gis_anchor  = {
        40.410262,
        -79.948172,
        1.0
    };
    virtual nlohmann::json dump(){
        return *this;
    }
};

void inline to_json(nlohmann::json& j, const CommandBaseConfig& config)
{
    j = nlohmann::json{
        {"type", config.type},
        {"listen_port", config.listen_port},
    };
};
void inline from_json(const nlohmann::json& j, CommandBaseConfig& config)
{
    json_get(j, "type", config.type);
    json_get(j, "listen_port", config.listen_port);
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