// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
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
    std::string replay_class;
    std::string color;
    Odometry odometry;
    std::vector<std::string> tags;
    std::vector<OOBB> oobbs;
};

struct CollisionTarget {
    Vec3f acceleration;
    float brake_input;
    bool collision;
    float distance;
    float forward_acceleration;
    float forward_velocity;
    std::string name;
    Vec3f relative_velocity;
    float throttle_input;
    float time_to_collision;
    Vec3f velocity;
    float wheel_input;
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

#pragma pack(push, 1)
struct LidarHit{
    uint16_t distance;
    uint8_t reflection;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct LidarBlock{
    uint16_t blockId;
    uint16_t azimuth;
    LidarHit hits[32];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct LidarPacket{
    LidarBlock blocks[12];
    uint32_t time_stamp;
    uint16_t packet_end;
    inline void set_start_block(int blockIndex, uint16_t blockId, uint16_t azimuth){
        blocks[blockIndex].blockId = blockId;
        blocks[blockIndex].azimuth = azimuth;
    }
    inline void set_hit(int blockIndex, int hitIndex, uint16_t distance, uint8_t reflection){
        blocks[blockIndex].hits[hitIndex].distance = distance;
        blocks[blockIndex].hits[hitIndex].reflection = reflection;
    }
    inline void set_end_packet(uint32_t timeStamp, uint16_t packetEnd){
        time_stamp = timeStamp;
        packet_end = packetEnd;
    }
};
#pragma pack(pop)

struct UltrasonicTarget{
    float range;
};

enum LaneChange {
	NONE = 0,
	Right = 1,
	Left = 2,
	Both = 3
};

struct Waypoint {
    Vec3f location;
    Vec3f rotation;
    float distance;
    int lane_change;
};

struct ActorLane {
    std::string road_id;
    int lane_id;
    std::vector<Waypoint> waypoints;
};

struct ActorWaypoints {
    std::string actor_id;
    std::string actor_road_id;
    int actor_lane_id;
    Waypoint actor_waypoint;
    std::vector<ActorLane> lanes;
    std::vector<ActorLane> left_lanes;
    std::vector<ActorLane> right_lanes;
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
void MONODRIVECORE_API to_json(nlohmann::json& j, const UltrasonicTarget& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, UltrasonicTarget& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const CollisionTarget& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, CollisionTarget& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const Waypoint& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, Waypoint& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const ActorWaypoints& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, ActorWaypoints& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const ActorLane& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, ActorLane& v);

