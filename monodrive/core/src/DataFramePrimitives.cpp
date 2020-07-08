// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#include "DataFramePrimitives.h"

void to_json(nlohmann::json& j, const Quat& v) {
	j = {
		{"x", v.x},
		{"y", v.y},
		{"z", v.z},
		{"w", v.w}
	};
}
void from_json(const nlohmann::json& j, Quat& v) {
	if (j.find("w") != j.end()) {
		json_get(j, "x", v.x);
		json_get(j, "y", v.y);
		json_get(j, "z", v.z);
		json_get(j, "w", v.w);
	}
	else {
		v.x = j[0].get<float>();
		v.y = j[1].get<float>();
		v.z = j[2].get<float>();
		v.w = j[3].get<float>();
	}
}

void to_json(nlohmann::json& j, const Vec3f& v) {
	j = {
		{"x", v.x},
		{"y", v.y},
		{"z", v.z}
	};
}
void from_json(const nlohmann::json& j, Vec3f& v) {
	if (j.find("x") != j.end()) {
		json_get(j, "x", v.x);
		json_get(j, "y", v.y);
		json_get(j, "z", v.z);
	}
	else {
		v.x = j[0].get<float>();
		v.y = j[1].get<float>();
		v.z = j[2].get<float>();
	}
}

void to_json(nlohmann::json& j, const Transform& v) {
	j = {
		{"position", v.position},
		{"orientation", v.orientation}
	};
}
void from_json(const nlohmann::json& j, Transform& v) {
	json_get(j, "orientation", v.orientation);
	json_get(j, "position", v.position);
}

void to_json(nlohmann::json& j, const OOBB& v) {
	j = {
		{"name", v.name},
		{"orientation", v.orientation},
		{"center", v.center},
		{"scale", v.scale},
		{"extents", v.extents}
	};
}
void from_json(const nlohmann::json& j, OOBB& v) {
	json_get(j, "name", v.name);
	json_get(j, "orientation", v.orientation);
	json_get(j, "center", v.center);
	json_get(j, "scale", v.scale);
	json_get(j, "extents", v.extents);
}

void to_json(nlohmann::json& j, const WheelState& v) {
	j = {
		{"id", v.id},
		{"speed", v.speed},
		{"pose", v.pose}
	};
}
void from_json(const nlohmann::json& j, WheelState& v) {
	json_get(j, "id", v.id);
	json_get(j, "speed", v.speed);
	json_get(j, "pose", v.pose);
}

void to_json(nlohmann::json& j, const Odometry& v) {
	j = {
		{"pose", v.pose},
		{"linear_velocity", v.linear_velocity},
		{"angular_velocity", v.angular_velocity}
	};
}
void from_json(const nlohmann::json& j, Odometry& v) {
	json_get(j, "pose", v.pose);
	json_get(j, "linear_velocity", v.linear_velocity);
	json_get(j, "angular_velocity", v.angular_velocity);
}

void to_json(nlohmann::json& j, const ObjectState& v) {
	j = {
		{"name", v.name},
		{"odometry", v.odometry},
		{"tags", v.tags},
		{"oriented_bounding_box", v.oobbs}
	};
}
void from_json(const nlohmann::json& j, ObjectState& v) {
	json_get(j, "name", v.name);
	json_get(j, "odometry", v.odometry);
	json_get(j, "tags", v.tags);
	json_get(j, "oriented_bounding_box", v.oobbs);
}

void to_json(nlohmann::json& j, const VehicleState& v) {
	j = {
		{"state", v.state},
		{"wheels", v.wheels}
	};
}
void from_json(const nlohmann::json& j, VehicleState& v) {
	json_get(j, "state", v.state);
	json_get(j, "wheels", v.wheels);
}

void to_json(nlohmann::json& j, const BoundingBox2D& v) {
	j = {
		{"name", v.name},
		{"2d_bounding_box", {v.xmin, v.xmax, v.ymin, v.ymax}}
	};
}
void from_json(const nlohmann::json& j, BoundingBox2D& v) {
	v.name = j["name"].get<std::string>();
	auto& box = j["2d_bounding_box"];
	v.xmin = box[0].get<float>();
	v.xmax = box[1].get<float>();
	v.ymin = box[2].get<float>();
	v.ymax = box[3].get<float>();
}

void to_json(nlohmann::json& j, const AnnotationFrame2D& v) {
	j = {
		{"name", v.name},
		{"2d_bounding_boxes", v.bounding_boxes_2d},
		{"oriented_bounding_boxes", v.oriented_bounding_boxes},
		{"tags", v.tags}
	};
}
void from_json(const nlohmann::json& j, AnnotationFrame2D& v) {
	json_get(j, "name", v.name);
	json_get(j, "2d_bounding_boxes", v.bounding_boxes_2d);
	json_get(j, "oriented_bounding_boxes", v.oriented_bounding_boxes);
	json_get(j, "tags", v.tags);
}

void to_json(nlohmann::json& j, const RadarTarget& v){
	j = {
		{"range", v.range},
		{"aoa", v.aoa},
		{"velocity", v.velocity},
		{"rcs", v.rcs},
		{"target_ids", v.target_ids}
	};
}

void from_json(const nlohmann::json& j, RadarTarget& v){
	json_get(j, "range", v.range);
	json_get(j, "aoa", v.aoa);
	json_get(j, "velocity", v.velocity);
	json_get(j, "rcs", v.rcs);
	json_get(j, "target_ids", v.target_ids);
}

void to_json(nlohmann::json& j, const UltrasonicTarget& v){
	j = {
		{"range", v.range}
	};
}

void from_json(const nlohmann::json& j, UltrasonicTarget& v){
	json_get(j, "range", v.range);
}

void to_json(nlohmann::json& j, const CollisionTarget& v){
	j = {
		{"acceleration", v.acceleration},
		{"brake_input", v.brake_input},
    {"collision", v.collision},
    {"distance", v.distance},
    {"forward_acceleration", v.forward_acceleration},
    {"forward_velocity", v.forward_velocity},
    {"name", v.name},
    {"relative_velocity", v.relative_velocity},
    {"throttle_input", v.throttle_input},
    {"time_to_collision", v.time_to_collision},
    {"velocity", v.velocity},
    {"wheel_input", v.wheel_input}
	};
}

void from_json(const nlohmann::json& j, CollisionTarget& v){
    json_get(j, "acceleration", v.acceleration);
    json_get(j, "brake_input", v.brake_input);
    json_get(j, "collision", v.collision);
    json_get(j, "distance", v.distance);
    json_get(j, "forward_acceleration", v.forward_acceleration);
    json_get(j, "forward_velocity", v.forward_velocity);
    json_get(j, "name", v.name);
    json_get(j, "relative_velocity", v.relative_velocity);
    json_get(j, "throttle_input", v.throttle_input);
    json_get(j, "time_to_collision", v.time_to_collision);
    json_get(j, "velocity", v.velocity);
    json_get(j, "wheel_input", v.wheel_input);
}

void to_json(nlohmann::json& j, const Waypoint& w){
	j = {
		{"location", w.location},
		{"rotation", w.rotation},
		{"road_id", w.road_id},
		{"lane_id", w.lane_id},
		{"distance", w.distance},
		{"lane_change", w.lane_change}};
}
void from_json(const nlohmann::json& j, Waypoint& w){
    json_get(j, "location", w.location); 
    json_get(j, "rotation", w.rotation); 
    json_get(j, "road_id", w.road_id); 
    json_get(j, "lane_id", w.lane_id); 
    json_get(j, "distance", w.distance); 
    json_get(j, "lane_change", w.lane_change); 
}

void to_json(nlohmann::json& j, const ActorWaypoints& w){
	j = {
		{"id", w.actor_id},
		{"waypoints", w.waypoints}
	};
}
void from_json(const nlohmann::json& j, ActorWaypoints& w){
    json_get(j, "id", w.actor_id); 
    json_get(j, "waypoints", w.waypoints); 
}
