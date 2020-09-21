// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once

#include "JsonHelpers.h"

struct Location {
    Location() {}
    Location(float x, float y, float z) : x(x), y(y), z(z) {}
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

struct Rotation {
    Rotation() {}
    Rotation(float yaw, float pitch, float roll) : yaw(yaw), pitch(pitch), roll(roll) {}
    float yaw{0.0f};
    float pitch{0.0f};
    float roll{0.0f};
};

struct Resolution {
  Resolution() {}
  Resolution(int x, int y) : x(x), y(y) {}
  int x{512};
  int y{512};
};

struct Color {
    Color() {}
    Color(int a, int r, int g, int b) : a(a), r(r), g(g), b(b) {}
    int a{255};
    int r{0};
    int g{0};
    int b{0};
;

struct Viewport
{
    bool enable_viewport{false};
    Resolution window_size{0, 0};
    Resolution window_offset{0, 0};
    bool fullscreen{false};
    int monitor_number{0};
    std::string monitor_name{""};
};

void inline to_json(nlohmann::json& j, const Location& location)
{
    j = nlohmann::json{
        {"x", location.x},
        {"y", location.y},
        {"z", location.z}
    };
}
void inline from_json(const nlohmann::json& j, Location& location)
{
    json_get(j, "x", location.x);
    json_get(j, "y", location.y);
    json_get(j, "z", location.z);
}

void inline to_json(nlohmann::json& j, const Rotation& rotation)
{
    j = nlohmann::json{
        {"yaw", rotation.yaw},
        {"pitch", rotation.pitch},
        {"roll", rotation.roll}
    };
}
void inline from_json(const nlohmann::json& j, Rotation& rotation)
{
    json_get(j,"yaw", rotation.yaw);
    json_get(j,"pitch", rotation.pitch);
    json_get(j, "roll", rotation.roll);
}

void inline to_json(nlohmann::json& j, const Resolution& resolution)
{
    j = nlohmann::json{
        {"x", resolution.x},
        {"y", resolution.y}
    };
}

void inline from_json(const nlohmann::json& j, Resolution& resolution)
{
    json_get(j, "x", resolution.x);
    json_get(j, "y", resolution.y);
}

void inline to_json(nlohmann::json& j, const Viewport& config)
{
    j["enable_viewport"] = config.enable_viewport;
    j["window_size"] = config.window_size;
    j["window_offset"] = config.window_offset;
    j["fullscreen"] = config.fullscreen;
    j["monitor_number"] = config.monitor_number;
    j["monitor_name"] = config.monitor_name;
}

void inline from_json(const nlohmann::json& j, Viewport& config)
{
    json_get(j, "enable_viewport", config.enable_viewport);
    json_get(j, "window_size", config.window_size);
    json_get(j, "window_offset", config.window_offset);
    json_get(j, "fullscreen", config.fullscreen);
    json_get(j, "monitor_number", config.monitor_number);
    json_get(j, "monitor_name", config.monitor_name);
}
