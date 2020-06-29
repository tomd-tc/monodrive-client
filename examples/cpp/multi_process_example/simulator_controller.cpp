#include <iostream>		  // std::cout
#include <string>

//monoDrive Includes
#include "Simulator.h"
#include "Configuration.h"  // holder for sensor, simulator, scenario, weather, and vehicle configurations

//Single Simulator Example
std::string server0_ip = "127.0.0.1";
int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;

int main(int argc, char** argv)
{
    // read JSON files from config directory
    Configuration config(
        "examples/config/simulator_straightaway.json",
        "examples/config/weather.json",
        "examples/config/scenario_ego_on_shoulder_straightaway.json"
    );
    // set to closed loop mode
    config.simulator["simulation_mode"] = 0;

    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    while(true)
    {
    }
    
    return 0;
}
