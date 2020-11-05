#include "carla/road/Road.h"
#include "carla/opendrive/OpenDriveParser.h"
#include <fstream>
#include <streambuf>

using namespace carla;

int main(){

    std::ifstream openDriveFile("examples/config/opendrive/Infinity.xodr");
    std::string openDriveFileString((std::istreambuf_iterator<char>(openDriveFile)),
                 std::istreambuf_iterator<char>());
    auto map = opendrive::OpenDriveParser::Load(openDriveFileString);
    auto topo = map->GenerateTopology();
    int count = 0;
    std::cout << "TOPO: " << std::endl;
    for(auto& points : topo){
        std::cout << points.first.road_id << " " <<  points.first.lane_id << " " << points.first.section_id << " " << points.first.s;
        std::cout << " :Point " << count++ << std::endl;
        std::cout << points.second.road_id << " " <<  points.second.lane_id << " " << points.second.section_id << " " << points.second.s << std::endl;
    }
    carla::geom::Vector3D location(0,0,0);
    auto wayPoint = map->GetClosestWaypointOnRoad(location);


    auto waypoint = wayPoint.get();
    int road_id_final = waypoint.road_id;
    for(int road_id = 0; road_id != road_id_final; road_id = waypoint.road_id){
        std::cout << "--------------------------" << std::endl;
        std::cout << waypoint.road_id << " " << waypoint.lane_id << " " << waypoint.s << " " << std::endl;
        waypoint.s = map->GetDistanceAtEndOfLane(waypoint);
        std::cout << waypoint.road_id << " " << waypoint.lane_id << " " << waypoint.s << " " << std::endl;
        auto wayPoints = map->GetSuccessors(waypoint);
        waypoint = wayPoints[0];
    }
    std::cout << "--------------------------" << std::endl;
    std::cout << waypoint.road_id << " " << waypoint.lane_id << " " << waypoint.s << " " << std::endl;
    waypoint.s = map->GetDistanceAtEndOfLane(waypoint);
    std::cout << waypoint.road_id << " " << waypoint.lane_id << " " << waypoint.s << " " << std::endl;


    std::cout << "map load complete" << std::endl;
    return 0;
}