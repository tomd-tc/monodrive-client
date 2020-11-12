#include "carla/road/Road.h"
#include "carla/opendrive/OpenDriveParser.h"
#include "carla/road/element/RoadInfoSpeed.h"
#include <fstream>
#include <streambuf>

using namespace carla;

void look_backward(
    road::element::Waypoint waypoint,
    double searchDistance,
    std::vector<std::pair<road::element::Waypoint, double>>& waypoints,
    boost::optional<road::Map>& map,
    double totalSearchDistance = 0)
{
    if(totalSearchDistance >= searchDistance){
        return;
    }
    waypoints.emplace_back(waypoint, totalSearchDistance);
    totalSearchDistance += map->GetDistanceToStartOfLane(waypoint);
    for(auto& predecessor : map->GetPredecessors(waypoint)){
        look_backward(predecessor, searchDistance, waypoints, map, totalSearchDistance);
    }
}

std::vector<road::element::Waypoint> create_path(
    road::element::Waypoint waypoint, 
    double maxDistance, 
    boost::optional<road::Map>& map)
{
    std::vector<road::element::Waypoint> waypoints;
    // loop till either the maximum distance is consumed
    // or we hit a dead end
    while(true){
        waypoints.emplace_back(waypoint);
        double pathLength = map->GetDistanceToEndOfLane(waypoint);
        // std::cout << "max distance " << maxDistance << " path length " << pathLength << std::endl;
        // have we reached the end of the planning horizon distance?
        if(pathLength > maxDistance){
            auto endPoint = waypoint;
            endPoint.s = endPoint.lane_id < 0
                ? waypoint.s + maxDistance
                : waypoint.s - maxDistance;
            waypoints.emplace_back(endPoint);
            return waypoints;
        }
        // decrement the distance by the path traveled
        maxDistance -= pathLength;
        // get the successor waypoint on the path
        auto successors = map->GetSuccessors(waypoint);
        if(successors.size()==0)
            return waypoints;
        waypoint = successors[0];
    }
}

double GetPathLength(const std::vector<road::element::Waypoint>& path, const boost::optional<road::Map>& map){
    // if there is only one point on path, it isn't a path, useful during construction
	if (path.size() < 2)
		return 0;
	double length = 0;
    for(int i = 0;i < path.size()-2; ++i){
		length += map->GetDistanceToEndOfLane(path[i]);
    }
	// the last point is on same lane as predecessor
    length += map->GetDistanceToEndOfLane(path.end()[-2]) - map->GetDistanceToEndOfLane(path.end()[-1]);
	return length;
}

std::vector<road::element::Waypoint> create_path_3(
    road::element::Waypoint waypoint, 
    double maxDistance, 
    boost::optional<road::Map>& map)
{
    std::vector<road::element::Waypoint> path;
    path.emplace_back(waypoint);
    // starts at 0 for a fresh path
    // maxDistance = maxDistance - GetPathLength(path, map);
    while(true){
        auto& waypoint = path.back();
        double pathLength = map->GetDistanceToEndOfLane(waypoint);
        // std::cout << "max distance " << maxDistance << " path length " << pathLength << std::endl;
        // have we reached the end of the planning horizon distance?
        if(pathLength > maxDistance){
            auto endPoint = waypoint;
            endPoint.s = endPoint.lane_id < 0
                ? waypoint.s + maxDistance
                : waypoint.s - maxDistance;
            path.emplace_back(endPoint);
            return path;
        }
        // decrement the distance by the path traveled
        maxDistance -= pathLength;
        // get the successor waypoint on the path
        auto successors = map->GetSuccessors(waypoint);
        if(successors.size()==0)
            return path;
        path.emplace_back(successors[0]);
    }
}

std::vector<road::element::Waypoint> create_path_2(
    road::element::Waypoint waypoint, 
    double maxDistance, 
    boost::optional<road::Map>& map)
{
    std::vector<road::element::Waypoint> path;
    path.emplace_back(waypoint);
    while(true){
        auto& lastWaypoint = path.back();
        double extensionDistance = maxDistance - GetPathLength(path, map);
		double distanceToEndOfLane = map->GetDistanceToEndOfLane(lastWaypoint);
        if (extensionDistance >= distanceToEndOfLane) {
            auto successors = map->GetSuccessors(lastWaypoint);
            if (successors.size()==0) {
                auto finalWaypoint = lastWaypoint;
                finalWaypoint.s = map->GetDistanceAtEndOfLane(finalWaypoint);
                path.push_back(finalWaypoint);
                lastWaypoint.s = map->GetDistanceAtEndOfLane(lastWaypoint);
                return path;
            } else {
                auto nextWaypoint = successors[0];
                path.push_back(successors[0]);
            }
        }
        else {
            path.push_back(map->forward(lastWaypoint, extensionDistance));
            return path;
        }
    }
}


int infinity()
{
    std::vector<road::element::Waypoint> path;
    boost::optional<road::Map> map;

    std::ifstream openDriveFile("examples/config/opendrive/Infinity.xodr");
    std::string openDriveFileString((std::istreambuf_iterator<char>(openDriveFile)),
                 std::istreambuf_iterator<char>());
    map = opendrive::OpenDriveParser::Load(openDriveFileString);
    if(!map)
        return -1;
    auto topo = map->GenerateTopology();
    int count = 0;
    std::cout << "TOPO: " << std::endl;
    for(auto& points : topo){
        auto waypoint = points.first;
        // std::cout << points.first.road_id << " " <<  points.first.lane_id << " " << points.first.section_id << " " << points.first.s;
        // std::cout << " :Point " << count++ << std::endl;
        // std::cout << points.second.road_id << " " <<  points.second.lane_id << " " << points.second.section_id << " " << points.second.s << std::endl;
        std::cout << waypoint.road_id << " " 
        << waypoint.lane_id << " " 
        << waypoint.s << " " 
        << map->GetDistanceAtEndOfLane(waypoint) << std::endl;
    }

    std::cout << "FULL LOOP" << std::endl;
    // carla::geom::Vector3D location(0,0,0);
    // auto wayPoint = map->GetClosestWaypointOnRoad(location);
    boost::optional<road::element::Waypoint> wayPoint{road::element::Waypoint()};
    wayPoint->road_id = 2;
    wayPoint->lane_id = -1;
    wayPoint->section_id = 0;
    wayPoint->s = 20.64;
    auto location = map->ComputeTransform(wayPoint.get()).location;
    {
        // loop around
        auto waypoint = wayPoint.get();
        int road_id_final = waypoint.road_id;
        for(int road_id = 0; road_id != road_id_final; road_id = waypoint.road_id){
            std::cout << waypoint.road_id << " " 
            << waypoint.lane_id << " " 
            << waypoint.s << " " 
            << map->GetDistanceAtEndOfLane(waypoint) << std::endl;
            auto wayPoints = map->GetSuccessors(waypoint);
            waypoint = wayPoints[0];
        }
        std::cout << waypoint.road_id << " " 
        << waypoint.lane_id << " " 
        << waypoint.s << " " 
        << map->GetDistanceAtEndOfLane(waypoint) << std::endl;
    }

    const double pathDistance = 700;

    // start fresh
    wayPoint = map->GetClosestWaypointOnRoad(location).get();

    {
        std::cout << "+++++++++++++  PATH  +++++++++++++" << std::endl;
        auto path = create_path(wayPoint.get(), pathDistance, map);
        for(auto& waypoint : path){
            std::cout << waypoint.road_id << " " << waypoint.lane_id << " " << waypoint.s << " " << std::endl;
        }
    }

    std::cout << "++++++++++++++ PATH FROM NEXT ++++++++++++++++" << std::endl;
    {
        auto wayPoints = map->GetNext(wayPoint.get(), pathDistance);
        for(auto& waypoint : wayPoints){
            std::cout << waypoint.road_id << " " << waypoint.lane_id << " " << waypoint.s << " " << std::endl;
        }
    }
    {
        std::cout << "+++++++++++++  PATH 2 +++++++++++++" << std::endl;
        auto path = create_path_2(wayPoint.get(), pathDistance, map);
        for(auto& waypoint : path){
            std::cout << waypoint.road_id << " " << waypoint.lane_id << " " << waypoint.s << " " << std::endl;
        }
    }
    {
        std::cout << "+++++++++++++  PATH 3 +++++++++++++" << std::endl;
        auto path = create_path_3(wayPoint.get(), pathDistance, map);
        for(auto& waypoint : path){
            std::cout << waypoint.road_id << " " << waypoint.lane_id << " " << waypoint.s << " " << std::endl;
        }
        std::cout << GetPathLength(path, map) << std::endl;
    }
    auto& Map = map.get();
    {
        std::cout << "+++++++++++++  SPEED +++++++++++++" << std::endl;
        auto path = create_path_3(wayPoint.get(), pathDistance, map);
        for(auto& waypoint : path){
            auto speed = map->GetRoad(waypoint).GetInfo<road::element::RoadInfoSpeed>(waypoint.s)->GetSpeed();
            std::cout << waypoint.road_id << " " << waypoint.lane_id << " " << waypoint.s << " " << speed << std::endl;
        }
        std::cout << GetPathLength(path, map) << std::endl;
    }
    {
        std::cout << "+++++++++++++  Backward Search +++++++++++++" << std::endl;
        std::vector<std::pair<road::element::Waypoint, double>> waypoins;
        look_backward(wayPoint.get(), pathDistance, waypoins, map);
        for(auto& point : waypoins){
            auto waypoint = point.first;
            std::cout << waypoint.road_id << " " << waypoint.lane_id << " " << waypoint.s << " " << point.second << std::endl;
        }
        // std::cout << GetPathLength(path, map) << std::endl;
    }



    return 0;
}

int cross_roads(){
    std::vector<road::element::Waypoint> path;
    boost::optional<road::Map> map;

    std::ifstream openDriveFile("examples/config/opendrive/SimpleJunction.xodr");
    std::string openDriveFileString((std::istreambuf_iterator<char>(openDriveFile)),
                 std::istreambuf_iterator<char>());
    map = opendrive::OpenDriveParser::Load(openDriveFileString);
    if(!map)
        return -1;
    


    return 0;
}

int main(){
    if(infinity())
        return -1;
    if(cross_roads())
        return -1;


    return 0;
}