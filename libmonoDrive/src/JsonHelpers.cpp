// Copyright 2017-2020 monoDrive, LLC. All Rights Reserved.
#include "JsonHelpers.h"

#if defined UE_BUILD_DEBUG || defined UE_BUILD_DEVELOPMENT || defined UE_BUILD_TEST || defined UE_BUILD_SHIPPING

void to_json(nlohmann::json& j, const FRotator& r) {
	j = nlohmann::json{ 
		{ "roll", r.Roll },
		{ "pitch", r.Pitch },
		{ "yaw", r.Yaw } 
	};
}

void from_json(const nlohmann::json& j, FRotator& v) {
	if (j.find("pitch") != j.end()) {
		v.Pitch = j.at("pitch").get<float>();
		v.Yaw = j.at("yaw").get<float>();
		v.Roll = j.at("roll").get<float>();
	}
	else {
		FQuat quat(
			j[0].get<float>(),
			j[1].get<float>(),
			j[2].get<float>(),
			j[3].get<float>());
		v = quat.Rotator();
	}
}

void to_json(nlohmann::json& j, const FVector& v) {
	j = nlohmann::json{ 
		{ "x", v.X },
		{ "y", v.Y },
		{ "z", v.Z } 
	};
}

void from_json(const nlohmann::json& j, FVector& v) {
	if (j.find("x") != j.end()) {
		v.X = j.at("x").get<float>();
		v.Y = j.at("y").get<float>();
		v.Z = j.at("z").get<float>();
	}
	else {
		auto vec = j.get<std::vector<float>>();
		v.X = j[0].get<float>();
		v.Y = j[1].get<float>();
		v.Z = j[2].get<float>();
	}
}

void to_json(nlohmann::json& j, const FVector2D& v) {
	j = nlohmann::json{ { "x", v.X },{ "y", v.Y } };
}

void from_json(const nlohmann::json& j, FVector2D& v) {
	if (j.find("x") != j.end()) {
		v.X = j.at("x").get<float>();
		v.Y = j.at("y").get<float>();
	}
	else {
		v.X = j[0].get<float>();
		v.Y = j[1].get<float>();
	}
}

void to_json(nlohmann::json& j, const FQuat& v) {
	j = nlohmann::json{
		{"x", v.X},
		{"y", v.Y},
		{"z", v.Z},
		{"w", v.W},
	};
}

void from_json(const nlohmann::json& j, FQuat& v) {
	if (j.find("w") != j.end()) {
		v = FQuat(
			j.at("x").get<float>(),
			j.at("y").get<float>(),
			j.at("z").get<float>(),
			j.at("w").get<float>()
		);
	}
	else {
		v = FQuat(
			j[0].get<float>(),
			j[1].get<float>(),
			j[2].get<float>(),
			j[3].get<float>()
		);
	}
}

void to_json(nlohmann::json& j, const FLinearColor& c) {
	j = nlohmann::json{ 
		{ "R", c.R },
		{ "G", c.G }, 
		{ "B", c.B }, 
		{ "A", c.A } 
	};
}

void from_json(const nlohmann::json& j, FLinearColor& c) {
	c.R = j.at("R").get<float>();
	c.G = j.at("G").get<float>();
	c.B = j.at("B").get<float>();
	c.A = j.at("A").get<float>();
}

#endif