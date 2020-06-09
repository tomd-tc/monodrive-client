#pragma once

#include "JsonHelpers.h"
#include <fstream>
#include <vector>
#include <Eigen/Eigen>
#include <tuple>

#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
/// this is a stub for future development, full lane spline graph and commands live server side
/// this is temporary to provide some functionality for demos, will be replaced by lanelet code

namespace lane_spline{

class LaneSpline{
public:
    LaneSpline(){}
    LaneSpline(const std::string& geoJsonFile);
    LaneSpline(const nlohmann::json& geoJson);
    void AddLane(const nlohmann::json& lane, const nlohmann::json& laneGeo);
    int GetNearestPoint(const std::string& road, const std::string& lane, const Eigen::VectorXd& point);
    // cv::Mat GetPointsWithinRadius(Eigen::VectorXd& query_point, double radius);
    std::map<std::string, std::map<std::string, std::vector<Eigen::VectorXd>>> spline_map;
private:
    // Member functions
    void ParseLaneSplines(const nlohmann::json& geoJson);
    // void BuildFlannSearch();
    // Data
    // std::map<std::string, std::map<std::string, cv::flann::Index>> spline_query_map;
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

void LaneSpline::AddLane(const nlohmann::json& lane, 
        const nlohmann::json& laneGeo){
    std::string road = lane["road"].get<std::string>();
    std::vector<Eigen::VectorXd> points;
    for(int i = 0; i < laneGeo["coordinates"].size(); i++) {
        Eigen::VectorXd location(3);
        location << laneGeo["coordinates"][i][0].get<double>(),
            laneGeo["coordinates"][i][1].get<double>(), 
            lane["z_values"][i];
        points.emplace_back(location);
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
            AddLane(feature["properties"], feature["geometry"]);
        }
    }

    // BuildFlannSearch();
}

// void LaneSpline::BuildFlannSearch() 
// {
//     cv::Mat_<float> features(0, 2);

//     for(auto& road : spline_map) {
//         for(auto& lane : road.second) {
//             for(auto& pt : lane.second) {
//                 cv::Mat row = (cv::Mat_<float>(1, 2) << pt.x() , pt.y());
//                 features.push_back(row);
//             }
//             cv::flann::Index flann_index(features, cv::flann::KDTreeIndexParams(1));
//             spline_query_map[road.first][lane.first] = flann_index;
//             features = cv::Mat_<int>(0, 2);
//         }
//     }
// }

// cv::Mat LaneSpline::GetPointsWithinRadius(
//     Eigen::VectorXd& query_point, double radius) {
//   cv::Mat indices, dists;
//   cv::Mat query = (cv::Mat_<float>(1, 2) << query_point.x(), query_point.y());

//   for(auto& road : spline_query_map) {
//       for(auto& lane : road.second) {
//           lane.second.radiusSearch(query, indices, dists, radius, 1000);
//       }
//       break;
//   }

//   return indices;
// }

}