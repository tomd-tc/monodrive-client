// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once

#pragma warning(push)
#pragma warning(disable : 4946)
#include "json.hpp"
#pragma warning(pop)

#include <iostream>
#include <string>

#include "UECompatability.h"


#ifndef UE_BUILD
void inline json_log(const std::string& error_message)
{
    std::cerr << error_message << std::endl;
};
#else
#include "LogHelper.h"

inline void json_log(const FString& error_message) {
	EGO_LOG(LogLoad, Warning, "%s", *error_message);
}

inline void json_log(const std::string& error_message) {
	EGO_LOG(LogLoad, Warning, "%s", UTF8_TO_TCHAR(error_message.c_str()));
}

inline void to_json(nlohmann::json& j, const FRotator& v) {
	j = nlohmann::json{ 
		{ "pitch", v.Pitch },
		{ "yaw", v.Yaw },
		{ "roll", v.Roll }
	};
}

inline void from_json(const nlohmann::json& j, FRotator& v) {
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

inline void to_json(nlohmann::json& j, const FIntPoint& v)
{
	j = nlohmann::json{ 
		{ "x", v.X },
		{ "y", v.Y }
	};
}

inline void from_json(const nlohmann::json& j, FIntPoint& v)
{
	if (j.find("x") != j.end()) {
		v.X = j.at("x").get<int>();
		v.Y = j.at("y").get<int>();
	}
	else {
		auto vec = j.get<std::vector<float>>();
		v.X = j[0].get<int>();
		v.Y = j[1].get<int>();
	}

}

inline void to_json(nlohmann::json& j, const FVector& v) {
	j = nlohmann::json{ 
		{ "x", v.X },
		{ "y", v.Y },
		{ "z", v.Z } 
	};
}

inline void from_json(const nlohmann::json& j, FVector& v) {
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

inline void to_json(nlohmann::json& j, const FVector2D& v) {
	j = nlohmann::json{ { "x", v.X },{ "y", v.Y } };
}

inline void from_json(const nlohmann::json& j, FVector2D& v) {
	if (j.find("x") != j.end()) {
		v.X = j.at("x").get<float>();
		v.Y = j.at("y").get<float>();
	}
	else {
		v.X = j[0].get<float>();
		v.Y = j[1].get<float>();
	}
}

inline void to_json(nlohmann::json& j, const FQuat& v) {
	j = nlohmann::json{
		{"x", v.X},
		{"y", v.Y},
		{"z", v.Z},
		{"w", v.W},
	};
}

inline void from_json(const nlohmann::json& j, FQuat& v) {
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

inline void to_json(nlohmann::json& j, const FLinearColor& v) {
	j = nlohmann::json{ 
		{ "R", v.R },
		{ "G", v.G }, 
		{ "B", v.B }, 
		{ "A", v.A } 
	};
}

inline void from_json(const nlohmann::json& j, FLinearColor& v) {
	v.R = j.at("R").get<float>();
	v.G = j.at("G").get<float>();
	v.B = j.at("B").get<float>();
	v.A = j.at("A").get<float>();
}

inline void to_json(nlohmann::json& j, const FString& v) {
	j = TCHAR_TO_UTF8(*v);
}

inline void from_json(const nlohmann::json& j, FString& v) {
	v = FString(UTF8_TO_TCHAR(j.get<std::string>().c_str()));
}

inline void to_json(nlohmann::json& j, const FName& v) {
	j = TCHAR_TO_UTF8(*v.ToString());
}
inline void from_json(const nlohmann::json& j, FName& v) {
	v = FName(UTF8_TO_TCHAR(j.get<std::string>().c_str()));
}

inline void to_json(nlohmann::json& j, const TArray<FString>& v) {
	auto temp = nlohmann::json::array();
	for (auto& val : v) {
		temp.push_back(nlohmann::json(val));
	}
	j = temp;
}
inline void from_json(const nlohmann::json& j, TArray<FString>& v) {
	for (auto& val : j) {
		v.Add(FString(val.get<std::string>().c_str()));
	}
}

inline void to_json(nlohmann::json& j, const TArray<FName>& v) {
	auto temp = nlohmann::json::array();
	for (auto& val : v) {
		temp.push_back(nlohmann::json(val));
	}
	j = temp;
}
inline void from_json(const nlohmann::json& j, TArray<FName>& v) {
	for (auto& val : j) {
		v.Add(FName(val.get<std::string>().c_str()));
	}
}
#endif

// update me, should do the at check and return false rather than rely on exception handling
// this is nice for debugging but causes slow down on normal operations where we expect the 
// at to fail but have a default or something that is used when this function returns false
template <typename T>
bool inline json_get(const nlohmann::json& data, const std::string& key, T& value) {
    T temp;
    try {
        temp = data.at(key).get<T>();
    }
    catch (const nlohmann::detail::out_of_range& e) {
        // key not found error
        json_log(std::string(e.what()));
        return false;
    }
    catch (const nlohmann::detail::type_error& e) {
        // type error
        json_log(std::string("key \"") + std::string(key.c_str()) + std::string("\" sent as wrong type ") + std::string(e.what()));
        return false;
    }
    catch (const std::exception& e) {
        // unknown error
        json_log(std::string("json exception ") + std::string(e.what()));
        return false;
    }
    value = temp;
    return true;
}

template <typename T>
bool inline json_get(const nlohmann::json& data, T& value) {
    T temp;
    try {
        temp = data.get<T>();
    }
    catch (const nlohmann::detail::out_of_range& e) {
        // key not found error
        json_log(std::string(e.what()));
        return false;
    }
    catch (const nlohmann::detail::type_error& e) {
        // type error
        json_log(std::string("Wrong type ") + std::string(e.what()));
        return false;
    }
    catch (const std::exception& e) {
        // unknown error
        json_log(std::string("json exception ") + std::string(e.what()));
        return false;
    }
    value = temp;
    return true;
}

template <typename T>
T json_get_value(const nlohmann::json& j, std::string name, T defaultValue)
{
	if (j.find(name) != j.cend())
	{
		return j.at(name).get<T>();
	}
	return defaultValue;
}

inline bool json_contains_name(const nlohmann::json& j, const std::string &name) {
	for (auto& obj : j) {
		if (obj["name"] == name) {
			return true;
		}
	}
	return false;
}

// for json arrays can use to see if all keys are present in the array
// variadic list so can include as many as desired
// n*m complexity, for large lists of keys used frequently it is better to
// use a log(n) lookup data structure like a set, this is only for small numbers of keys
template<typename T>
bool json_contains(const nlohmann::json& j, T key) {
	return std::find(j.begin(), j.end(), key) != j.end();
}

template<typename T, typename... Args>
bool json_contains(const nlohmann::json& j, T key, Args... keys) {
	return std::find(j.begin(), j.end(), key) != j.end() && json_contains(j, keys...);
}

// for json arrays can use to see if at least one key is present in the array from a list of keys
// variadic list so can include as many as desired
template<typename T>
bool json_contains_one(const nlohmann::json& j, T key) {
	return std::find(j.begin(), j.end(), key) != j.end();
}

template<typename T, typename... Args>
bool json_contains_one(const nlohmann::json& j, T key, Args... keys) {
	return std::find(j.begin(), j.end(), key) != j.end() || json_contains_one(j, keys...);
}
