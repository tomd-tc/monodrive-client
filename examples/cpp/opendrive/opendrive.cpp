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
    for(auto& points : topo){
        // if(points.first.road_id == 0){
            std::cout << points.first.road_id << " " <<  points.first.lane_id << " " << points.first.section_id << " " << points.first.s;
            std::cout << " :Point " << count++ << std::endl;
            std::cout << points.second.road_id << " " <<  points.second.lane_id << " " << points.second.section_id << " " << points.second.s << std::endl;
        // }
    }
    carla::geom::Vector3D location(0,0,0);
    auto wayPoint = map->GetClosestWaypointOnRoad(location);
    auto nextWaypoints = map->GetSuccessors(wayPoint.get());
    std::cout << " NEXT " << std::endl;
    for(auto& next : nextWaypoints){
        std::cout << next.s << " " << next.road_id << " " << next.lane_id << std::endl;
    }

	auto getEndWaypoint = [&map](carla::road::element::Waypoint inWaypoint) {
		if (inWaypoint.lane_id < 0) {
			return carla::road::element::Waypoint{
				inWaypoint.road_id,
				inWaypoint.section_id,
				inWaypoint.lane_id,
				map->GetLane(inWaypoint).GetDistance()
			};
		}
		else {
			return carla::road::element::Waypoint{
				inWaypoint.road_id,
				inWaypoint.section_id,
				inWaypoint.lane_id,
				0
			};
		}
	};
    auto endPoint = getEndWaypoint(wayPoint.get());
    std::cout << endPoint.s << " " << endPoint.road_id << " " << endPoint.lane_id << std::endl;




 

    std::cout << "map load complete" << std::endl;
    return 0;
}