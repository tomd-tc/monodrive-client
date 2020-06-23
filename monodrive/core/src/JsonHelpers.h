// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once

#pragma warning(push)
#pragma warning(disable : 4946)
#include "json.hpp"
#pragma warning(pop)

#include <string>


#if defined UE_BUILD_DEBUG || defined UE_BUILD_DEVELOPMENT || defined UE_BUILD_TEST || defined UE_BUILD_SHIPPING
#include "LogHelper.h"

void MONODRIVECORE_API json_log(const FString& error_message);

void MONODRIVECORE_API json_log(const std::string& error_message);

void MONODRIVECORE_API to_json(nlohmann::json& j, const FName& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, FName& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const FString& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, FString& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const FRotator& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, FRotator& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const FQuat& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, FQuat& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const FVector& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, FVector& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const FVector2D& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, FVector2D& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const FLinearColor& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, FLinearColor& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const TArray<FString>& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, TArray<FString>& v);
void MONODRIVECORE_API to_json(nlohmann::json& j, const TArray<FName>& v);
void MONODRIVECORE_API from_json(const nlohmann::json& j, TArray<FName>& v);

#else
void inline json_log(const std::string& error_message)
{
    std::cerr << error_message << std::endl;
};

#endif

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

