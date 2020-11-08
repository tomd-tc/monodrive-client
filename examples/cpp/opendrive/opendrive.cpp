#include "carla/road/Road.h"
#include "carla/opendrive/OpenDriveParser.h"
#include <fstream>
#include <streambuf>

using namespace carla;

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
        std::cout << "max distance " << maxDistance << " path length " << pathLength << std::endl;
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



    // distanceTraveled += map->GetDistanceToEndOfLane(waypoint);
    // if(distanceTraveled > maxDistance){
    //     auto endPoint = waypoint;
    //     endPoint.s = endPoint.lane_id 
    //         ? waypoint.s + distanceTraveled - maxDistance
    //         : waypoint.s - (distanceTraveled - maxDistance);
    // }
    // do{
    //     auto successors = map->GetSuccessors(waypoint);
    //     if(successors.size() == 0){
    //         std::cerr << "The path hit a deadend." << std::endl;
    //         return waypoints;
    //     }
    //     waypoint = successors[0];
    //     waypoints.emplace_back(waypoint);
    //     distanceTraveled += map->GetDistanceToEndOfLane(waypoint);
    // } while(distanceTraveled < maxDistance);
    // waypoints.back().s = waypoints.back().lane_id < 0 
    //     ? waypoints.back().s + distanceTraveled - maxDistance
    //     : distanceTraveled - maxDistance;
    // return waypoints;
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
    // std::cout << "TOPO: " << std::endl;
    // for(auto& points : topo){
    //     std::cout << points.first.road_id << " " <<  points.first.lane_id << " " << points.first.section_id << " " << points.first.s;
    //     std::cout << " :Point " << count++ << std::endl;
    //     std::cout << points.second.road_id << " " <<  points.second.lane_id << " " << points.second.section_id << " " << points.second.s << std::endl;
    // }

    std::cout << "FULL LOOP" << std::endl;
    carla::geom::Vector3D location(0,0,0);
    auto wayPoint = map->GetClosestWaypointOnRoad(location);
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

    const double pathDistance = 450;

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