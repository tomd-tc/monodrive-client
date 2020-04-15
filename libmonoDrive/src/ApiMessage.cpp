
#include "ApiMessage.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdint.h>






void to_json(nlohmann::json& j, ApiMessage& m) {
	j = nlohmann::json{
		{ "reference", m.get_reference() },
		{ "type", m.get_message_type() },
		{ "success", m.get_success() },
		{ "message", m.get_message() }
	};
}

void from_json(const nlohmann::json& j, ApiMessage& m) {
	m.set_reference(json_get_value<long>(j, "reference", 0));
	m.set_message_type(j.at("type").get<std::string>());
	m.set_success(json_get_value<bool>(j, "success", false));
	m.set_message(j.at("message"));
}


//bool operator==(const ApiMessage& lhs, const std::string& rhs) {
//	return lhs.type.compare(rhs) == 0;
//}
//bool operator==(const std::string& lhs, const ApiMessage& rhs) {
//	return rhs == lhs;
//}
//bool operator==(const ApiMessage& lhs, const char* rhs) {
//	return lhs.type.compare(rhs) == 0;
//}
//bool operator==(const char* lhs, const ApiMessage& rhs) {
//	return rhs == lhs;
//}


ApiMessage ApiMessage::make_error(std::string error)
{
	return ErrorMessage(*this, error);
}


