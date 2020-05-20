#include <fstream>
#include <iostream>
#include <string>
#include "cxxopts.hpp"

#include "Configuration.h"


Configuration::Configuration(
    const fs::path& simulatorConfigPath,
    const fs::path& vehicleConfigPath,
    const fs::path& weatherConfigPath,
    const fs::path& scenarioConfigPath
){
	simulator = load(simulatorConfigPath);
    vehicle   = load(vehicleConfigPath);
    weather   = load(weatherConfigPath);
    scenario  = load(scenarioConfigPath);
}

// Configuration::Configuration(int argc, char** argv)
// {
// 	std::string simulator_config_path = "config/simulator.json";
// 	std::string vehicle_config_path = "config/vehicle.json";
// 	std::string sensor_config_path = "config/sensors.json";
// 	std::string weather_config_path = "config/weather.json";
// 	std::string scenario_config_path = "config/scenario.json";

// 	cxxopts::Options options("monoDrive Simulator", "controls the monoDrive simulator.");
// 	options.show_positional_help();
// 	options.add_options()
// 		("c,sim", "Simulator Config File", cxxopts::value<std::string>())
// 		("v,vehicle", "Vehicle Configuration", cxxopts::value<std::string>())
// 		("s,sensors", "Sensors Configuration File", cxxopts::value<std::string>())
// 		("w,weather", "Weather Configuration File")
// 		("e,scenario", "Scenario File", cxxopts::value<std::string>())
// 		("h,help", "Print Help")
// 		;
// 	auto cla = options.parse(argc, argv);
// 	if(cla.count("h")){
// 		std::cerr << options.help() << std::endl;
// 		throw std::runtime_error("Help called.");
// 	}
// 	if(cla.count("c")){
// 		simulator_config_path = cla["c"].as<std::string>();
// 	}
// 	if(cla.count("v")){
// 		vehicle_config_path = cla["v"].as<std::string>();
// 	}
// 	if(cla.count("s")){
// 		sensor_config_path = cla["s"].as<std::string>();
// 	}
// 	if(cla.count("w")){
// 		weather_config_path = cla["w"].as<std::string>();
// 	}
// 	if(cla.count("e")){
// 		scenario_config_path = cla["e"].as<std::string>();

// 	}

//     simulator = load(simulator_config_path);
//     vehicle   = load(vehicle_config_path);
//     weather   = load(weather_config_path);
//     scenario  = load(scenario_config_path);
// }

nlohmann::json Configuration::load(const fs::path& path)
{
    nlohmann::json j;
    std::cout << path.string() << std::endl;
    try
    {
        std::ifstream in(path, std::ifstream::in);
        in >> j;
        in.close();
    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
	
	return j;
}
