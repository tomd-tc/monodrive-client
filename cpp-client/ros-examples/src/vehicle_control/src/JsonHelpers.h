// Copyright 2017-2018 monoDrive, LLC. All Rights Reserved.
#pragma once

#pragma warning(push)
#pragma warning(disable : 4946)
#include "json.hpp"
#pragma warning(pop)

class JsonSerializable {
	virtual void to_json(nlohmann::json& j) const = 0;
	virtual void from_json(const nlohmann::json& j) = 0;
};

template <typename T, typename std::enable_if<std::is_base_of<JsonSerializable, T>::value>::type* = nullptr>
void to_json(nlohmann::json& j, const T& bm) {
	bm.to_json(j);
};

template <typename T, typename std::enable_if<std::is_base_of<JsonSerializable, T>::value>::type* = nullptr>
void from_json(const nlohmann::json& j, T& bm) {
	bm.from_json(j);
};



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

