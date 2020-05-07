#pragma once

#include "JsonHelpers.h"
#include <fstream>
#include <vector>
#include <Eigen/Eigen>
#include <tuple>

/// this is a stub for future development, full lane spline graph and commands live server side
/// this is temporary to provide some functionality for demos, will be replaced by lanelet code

namespace lane_spline{

class LaneSpline{
public:
    LaneSpline(){}
    LaneSpline(const std::string& geoJsonFile);
    LaneSpline(const nlohmann::json& geoJson);
    void AddLane(const nlohmann::json& lane);
    int GetNearestPoint(const std::string& road, const std::string& lane, const Eigen::VectorXd& point);
    std::map<std::string, std::map<std::string, std::vector<Eigen::VectorXd>>> spline_map;
private:
    void ParseLaneSplines(const nlohmann::json& geoJson);
};

int LaneSpline::GetNearestPoint(const std::string& road, const std::string& lane, const Eigen::VectorXd& point){
    double nearestDistance = std::numeric_limits<double>::max();
    int nearestPoint = -1;
    auto& points = spline_map[road][lane];
    for(int i = 0; i < points.size(); ++i)
    {
        auto& location = points[i];
        double distance = (location - point).norm();
        if(distance < nearestDistance){
            nearestDistance = distance;
            nearestPoint = i;
        }
    }
    return nearestPoint;
}

void LaneSpline::AddLane(const nlohmann::json& lane){
    std::string road = lane["road"].get<std::string>();
    std::vector<Eigen::VectorXd> points;
    for(auto& point : lane["points"]){
        Eigen::VectorXd location(3);
        location << point["location"]["x"].get<double>(),
            point["location"]["y"].get<double>(),
            point["location"]["z"].get<double>();
        points.push_back(location);
    }
    spline_map[road][lane["id"].get<std::string>()] = points;
}

LaneSpline::LaneSpline(const std::string& geoJsonFile){
    std::ifstream jsonFile(geoJsonFile);
    nlohmann::json geoJson;
    jsonFile >> geoJson;
    ParseLaneSplines(geoJson);
}

LaneSpline::LaneSpline(const nlohmann::json& geoJson) {
    ParseLaneSplines(geoJson);
}

void LaneSpline::ParseLaneSplines(const nlohmann::json& geoJson) {
    auto& features = geoJson["map"]["features"];
    for(auto& feature : features){
        if(feature["properties"]["feature_type"] == "road"){
            continue;
        }
        else if(feature["properties"]["feature_type"] == "lane"){
            AddLane(feature["properties"]);
        }
    }
}

}