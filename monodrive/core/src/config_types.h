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

void inline to_json(nlohmann::json& j, const Location& location)
{
    j = nlohmann::json{{"x", location.x},
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