// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#include "JsonHelpers.h"

#if defined UE_BUILD_DEBUG || defined UE_BUILD_DEVELOPMENT || defined UE_BUILD_TEST || defined UE_BUILD_SHIPPING
#include "monoDriveCore.h"

void json_log(const FString& error_message) {
	EGO_LOG(LogMD_Core, Warning, "%s", *error_message);
}

void json_log(const std::string& error_message) {
	EGO_LOG(LogMD_Core, Warning, "%s", UTF8_TO_TCHAR(error_message.c_str()));
}

void to_json(nlohmann::json& j, const FRotator& v) {
	j = nlohmann::json{ 
		{ "pitch", v.Pitch },
		{ "yaw", v.Yaw },
		{ "roll", v.Roll }
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

void to_json(nlohmann::json& j, const FLinearColor& v) {
	j = nlohmann::json{ 
		{ "R", v.R },
		{ "G", v.G }, 
		{ "B", v.B }, 
		{ "A", v.A } 
	};
}

void from_json(const nlohmann::json& j, FLinearColor& v) {
	v.R = j.at("R").get<float>();
	v.G = j.at("G").get<float>();
	v.B = j.at("B").get<float>();
	v.A = j.at("A").get<float>();
}

void to_json(nlohmann::json& j, const FString& v) {
	j = TCHAR_TO_UTF8(*v);
}

void from_json(const nlohmann::json& j, FString& v) {
	v = FString(UTF8_TO_TCHAR(j.get<std::string>().c_str()));
}

void to_json(nlohmann::json& j, const FName& v) {
	j = TCHAR_TO_UTF8(*v.ToString());
}
void from_json(const nlohmann::json& j, FName& v) {
	v = FName(UTF8_TO_TCHAR(j.get<std::string>().c_str()));
}

void to_json(nlohmann::json& j, const TArray<FString>& v) {
	auto temp = nlohmann::json::array();
	for (auto& val : v) {
		temp.push_back(nlohmann::json(val));
	}
	j = temp;
}
void from_json(const nlohmann::json& j, TArray<FString>& v) {
	for (auto& val : j) {
		v.Add(FString(val.get<std::string>().c_str()));
	}
}

void to_json(nlohmann::json& j, const TArray<FName>& v) {
	auto temp = nlohmann::json::array();
	for (auto& val : v) {
		temp.push_back(nlohmann::json(val));
	}
	j = temp;
}
void from_json(const nlohmann::json& j, TArray<FName>& v) {
	for (auto& val : j) {
		v.Add(FName(val.get<std::string>().c_str()));
	}
}
#endif