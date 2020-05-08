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
		{"2d_bounding_boxes", {v.xmin, v.xmax, v.ymin, v.ymax}}
	};
}
void from_json(const nlohmann::json& j, BoundingBox2D& v) {
	// v.name = j["name"].get<std::string>();
	// auto& box = j["2d_bounding_boxes"];
	// v.xmin = box[0].get<float>();
	// v.xmax = box[1].get<float>();
	// v.ymin = box[2].get<float>();
	// v.ymax = box[3].get<float>();
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