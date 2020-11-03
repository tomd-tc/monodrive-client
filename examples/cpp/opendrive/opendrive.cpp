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

    std::cout << "map load complete" << std::endl;
    return 0;
}