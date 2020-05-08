#pragma once

#include "JsonHelpers.h"
#include <algorithm>
#include "UECompatability.h"

struct Quat{
    float x, y, z, w;
};

struct Vec3f{
    float x, y, z;
};

struct Transform{
    Vec3f position;
    Quat orientation;
};

struct OOBB{
    std::string name;
    Quat orientation;
    Vec3f center;
    Vec3f scale;
    Vec3f extents;
};

struct WheelState{
    int32_t id;
    float speed;
    Transform pose;
};

struct Odometry{
    Transform pose;
    Vec3f linear_velocity;
    Vec3f angular_velocity;
};

struct ObjectState{
    std::string name;
    Odometry odometry;
    std::vector<std::string> tags;
    std::vector<OOBB> oobbs;
};

struct VehicleState{
    ObjectState state;
    std::vector<WheelState> wheels;
};

struct BoundingBox2D {
	std::string name;
	float xmin, xmax, ymin, ymax;
};

struct AnnotationFrame2D {
	std::string name;
	std::vector<BoundingBox2D> bounding_boxes_2d;
	std::vector<OOBB> oriented_bounding_boxes;
	std::vector<std::string> tags;
};

struct RadarTarget{
    float range;
    float aoa;
    float velocity;
    float rcs;
    std::vector<std::string> target_ids;
};

void MONODRIVECORE_API to_json(nlohmann::json& j, const Quat& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, Quat& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const Vec3f& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, Vec3f& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const Transform& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, Transform& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const OOBB& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, OOBB& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const WheelState& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, WheelState& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const Odometry& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, Odometry& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const ObjectState& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, ObjectState& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const VehicleState& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, VehicleState& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const BoundingBox2D& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, BoundingBox2D& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const AnnotationFrame2D& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, AnnotationFrame2D& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const RadarTarget& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, RadarTarget& v);




