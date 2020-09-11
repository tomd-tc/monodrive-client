#include <iostream>

//monoDrive Includes
#include "Simulator.h"
#include "Configuration.h"
#include "Sensor.h"
#include "sensor_config.h"

int main(int argc, char** argv)
{
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;

    //Read JSON files in cpp_client/config directory
    Configuration config(
        "examples/config/simulator_straightaway.json",
        "examples/config/weather.json",
        "examples/config/scenario_multi_vehicle_straightaway.json"
    );
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;
    WaypointConfig wp_config;
    wp_config.server_ip = sim0.getServerIp();
    wp_config.server_port = sim0.getServerPort();
    wp_config.listen_port = 8102;
    wp_config.distance = 10000;
    wp_config.frequency = 100;
    wp_config.draw_debug = true;
    wp_config.debug_tags = {"ego"};
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<WaypointConfig>(wp_config)));

    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim0.getServerIp();
    vp_config.server_port = sim0.getServerPort();
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256,256);
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    // Send configurations to the simulator
    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }

    /// initialize the vehicle, the first control command spawns the vehicle
    sim0.sendControl(0.1, 0.0, 0.0, 1);

    sensors[0]->sampleCallback = [](DataFrame* frame){
        auto& wpFrame = *static_cast<WaypointFrame*>(frame);
        for(const auto& actor : wpFrame.actor_waypoints) {
            std::cout << "Actor: " << actor.actor_id << " has " 
                << actor.lanes.size() << " lane(s)." << std::endl;
            std::cout << "\tActor road: " << actor.actor_road_id 
                        << " Actor lane: " << actor.actor_lane_id  << " - "
                        << actor.actor_waypoint.distance << ", " 
                        << actor.actor_waypoint.location.x << ", " 
                        << actor.actor_waypoint.location.y << ", " 
                        << actor.actor_waypoint.location.z << std::endl;
            for(const auto& lane : actor.lanes) {
                if(lane.waypoints.size() >= 2) {
                    std::cout << "\tLane waypoints: " << lane.road_id << " - " << lane.lane_id << " Count: " << lane.waypoints.size() << std::endl;
                    std::cout << "\t\tWP 0: " 
                        << lane.waypoints[0].distance << ", " 
                        << lane.waypoints[0].location.x << ", " 
                        << lane.waypoints[0].location.y << ", " 
                        << lane.waypoints[0].location.z << std::endl;
                    std::cout << "\t\tWP 1: "
                        << lane.waypoints[1].distance << ", " 
                        << lane.waypoints[1].location.x << ", " 
                        << lane.waypoints[1].location.y << ", " 
                        << lane.waypoints[1].location.z << std::endl;
                }
            }
        }
    };

    std::cout << "Sampling sensor loop" << std::endl;
    while(true)
    {
        sim0.sampleAll(sensors);
    }
    
    return 0;
}

