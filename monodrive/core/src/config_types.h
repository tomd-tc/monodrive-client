// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once

#include "JsonHelpers.h"

struct Resolution {
  Resolution() {}
  Resolution(int x, int y) : x(x), y(y) {}
  int x{512};
  int y{512};
};

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