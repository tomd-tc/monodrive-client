// Copyright 2017-2020 monoDrive, LLC. All Rights Reserved.

#include "Jobs.h"

Job::Job(int argc, char** argv)
{
    parseArguments(argc, argv);
}

bool Job::setResult(const Result& result)
{

}

void Job::parseArguments(int argc, char** argv)
{

}

nlohmann::json Result::dump()
{
    return *this;
}

void to_json(nlohmann::json& j, const Result& r)
{
    j["pass"] = r.pass;
    j["message"] = r.message;
}
void from_json(const nlohmann::json& j, Result& r)
{
    json_get(j, "pass", r.pass);
    json_get(j, "message", r.message);
}
