#include <iostream> // std::cout

#include "cxxopts.hpp"

#include "Simulator.h"
#include "Configuration.h" // holder for sensor, simulator, scenario, weather, and vehicle configurations
#include "Sensor.h"
#include "sensor_config.h"
#include "Stopwatch.h"


// defaults
int iterations = 64;
std::string workload = "full";
std::string mode = "closed_loop";
std::string server_ip = "127.0.0.1";
int server_port = 8999;


// helper to create a single camera sensor suite
std::vector<std::shared_ptr<Sensor>> create_camera_sensors(const std::string &ip)
{
	std::vector<std::shared_ptr<Sensor>> sensors;

	CameraConfig cam_config;
	cam_config.server_ip = ip;
	cam_config.listen_port = 8100;
	cam_config.location.z = 200;
	cam_config.rotation.pitch = -5;
	cam_config.resolution.x = 1280;
	cam_config.resolution.y = 800;

	sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(cam_config)));

	return sensors;
}


// helper to create a full sensor suite
std::vector<std::shared_ptr<Sensor>> create_full_sensors(const std::string &ip)
{
	std::vector<std::shared_ptr<Sensor>> sensors;

	// camera
	CameraConfig cam_config;
	cam_config.server_ip = ip;
	cam_config.listen_port = 8100;
	cam_config.location.z = 200;
	cam_config.rotation.pitch = -5;
	cam_config.resolution.x = 512;
	cam_config.resolution.y = 512;
	sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(cam_config)));

	// lidar
	LidarConfig lidar_config;
	lidar_config.server_ip = ip;
	lidar_config.listen_port = 8200;
	lidar_config.location.z = 200;
	sensors.push_back(std::make_shared<Sensor>(std::make_unique<LidarConfig>(lidar_config)));

	// radar
	RadarConfig radar_config;
	radar_config.server_ip = ip;
	radar_config.listen_port = 8300;
	radar_config.location.x = 250;
	radar_config.location.z = 50;
	sensors.push_back(std::make_shared<Sensor>(std::make_unique<RadarConfig>(radar_config)));

	// ultrasonic
	UltrasonicConfig us_config;
	us_config.server_ip = ip;
	us_config.listen_port = 8400;
	us_config.location.x = 300.f;
	us_config.location.z = 50.f;
	sensors.push_back(std::make_shared<Sensor>(std::make_unique<UltrasonicConfig>(us_config)));

	// gps
	GPSConfig gps_config;
	gps_config.server_ip = ip;
	gps_config.listen_port = 8500;
	sensors.push_back(std::make_shared<Sensor>(std::make_unique<GPSConfig>(gps_config)));

	// imu
	IMUConfig imu_config;
	imu_config.server_ip = ip;
	imu_config.listen_port = 8600;
	sensors.push_back(std::make_shared<Sensor>(std::make_unique<IMUConfig>(imu_config)));

	return sensors;
}


int parse_options(int argc, char* argv[])
{
    try {
        cxxopts::Options options("monoDrive benchmark", "Evaluates performance of monoDrive simulator on common workloads.");
        options.show_positional_help();
        options.add_options()
            ("i,iterations", "Number of iterations", cxxopts::value<int>())
            ("w,workload", "Workload option (camera or full)", cxxopts::value<std::string>())
            ("m,mode", "Simulator mode (closed_loop or replay)", cxxopts::value<std::string>())
			("c,config_file", "Path to config file (not currently used)", cxxopts::value<std::string>())
            ("h,help", "Print help")
            ;
        auto cla = options.parse(argc, argv);
        if (cla.count("h")) {
            std::cerr << options.help() << std::endl;
            return -1;
        }
		if (cla.count("i"))
		{
			try
			{
				iterations = cla["i"].as<int>();
			}
			catch (const std::exception& e)
			{
				std::cerr << e.what() << '\n';
			}
		}
		if (cla.count("w"))
		{
			try
			{
				workload = cla["w"].as<std::string>();
			}
			catch (const std::exception& e)
			{
				std::cerr << e.what() << '\n';
			}
		}
		if (cla.count("m"))
		{
			try
			{
				mode = cla["m"].as<std::string>();
			}
			catch (const std::exception& e)
			{
				std::cerr << e.what() << '\n';
			}
		}
	}
	catch (const cxxopts::OptionException& e)
	{
		std::cerr << "Error Parsing options: " << e.what() << std::endl;
		return -1;
	}
	return 0;
}

// main
int main(int argc, char* argv[])
{
	if (parse_options(argc, argv)) {
		return -1;
	}

	// setup
	float total_time = 0;
	int total_steps = 0;

	for (int it = 0; it < iterations; it++)
	{
		// configure simulator from local files
		Configuration config(
			"examples/config/simulator.json",
			"examples/config/weather.json",
			mode == "closed_loop"
            ? "examples/config/scenario_multi_vehicle_almono.json"
            : "examples/config/scenario.json"
		);
		config.simulator["simulation_mode"] = mode == "closed_loop" ? 0 : 2;
		Simulator& sim = Simulator::getInstance(config, server_ip, server_port);
		if (!sim.configure()) {
			return -1;
		}

		// setup and connect sensors
		std::vector<std::shared_ptr<Sensor>> sensors;
		if (workload == "full") {
			sensors = create_full_sensors(server_ip);
		} else if (workload == "camera") {
			sensors = create_camera_sensors(server_ip);
		} else {
			std::cout << "Unknown workload: " << workload << std::endl;
			return -1;
		}
		for (auto &sensor : sensors)
		{
			sensor->configure();
		}

		// setup and start timing
		int idx = 0;
		mono::precise_stopwatch stopwatch;

		if (mode == "closed_loop")
		{
			// initialize vehicle with first control command
			sim.sendControl(0, 0, 1, 1);

			// sample sensors as many times as possible over 10 seconds
			while (stopwatch.elapsed_time<unsigned int, std::chrono::seconds>() < 10.0)
			{
				sim.sampleAll(sensors);
				idx++;
			}
		}
		else if (mode == "replay")
		{
			// step through scenario while sampling sensors
			int nSteps = config.scenario.size();
			for (idx; idx < nSteps; idx++)
			{
				sim.stepSampleAll(sensors, idx, 1);
			}
		}

		// calculate FPS
		auto scenario_time = stopwatch.elapsed_time<unsigned int, std::chrono::microseconds>();
		auto fps = float(idx) / scenario_time * 1000000.0;
		std::cout << "iter: " << it << " fps = " + std::to_string(fps) << std::endl;
		total_time += scenario_time;
		total_steps += idx;

		std::this_thread::sleep_for(std::chrono::milliseconds(3000));

		// cleanup
		for (auto &sensor : sensors)
		{
			sensor->stopListening();
		}
		sim.stop();
		Simulator::clearInstances();
	}
	// overall fps
	auto fps = float(total_steps) / total_time * 1000000.0;
	std::cout << "overall fps = " + std::to_string(fps) << std::endl;

	return 0;
}
