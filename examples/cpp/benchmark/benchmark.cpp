#include <iostream> // std::cout

#include "cxxopts.hpp"

#include "Simulator.h"
#include "Configuration.h" // holder for sensor, simulator, scenario, weather, and vehicle configurations
#include "Sensor.h"
#include "sensor_config.h"
#include "Stopwatch.h"


// defaults
int iterations = 64;
std::string sensors_path = "./examples/cpp/benchmark/workload_full.json";
std::string mode = "closed_loop";
std::string server_ip = "127.0.0.1";
int server_port = 8999;

// helper function to parse command line arguments
int parse_options(int argc, char* argv[])
{
    try {
        cxxopts::Options options("monoDrive benchmark", "Evaluates performance of monoDrive simulator on common workloads.");
        options.show_positional_help();
        options.add_options()
            ("i,iterations", "Number of iterations", cxxopts::value<int>())
            ("s,sensors", "Path to sensor workload configuration", cxxopts::value<std::string>())
            ("m,mode", "Simulator mode (closed_loop or replay)", cxxopts::value<std::string>())
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
		if (cla.count("s"))
		{
			try
			{
				sensors_path = cla["s"].as<std::string>();
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
		// define configuration for benchmark
		Configuration config(
			"examples/config/simulator_no_traffic.json",
			"examples/config/weather.json",
			mode == "closed_loop"
            ? "examples/config/scenario_multi_vehicle_almono.json"
            : "examples/config/scenario.json",
			sensors_path
		);
		config.simulator["simulation_mode"] = mode == "closed_loop" ? 0 : 2;

		// load sensor workload
		std::vector<std::shared_ptr<Sensor>> sensors;
		config.loadSensors(sensors);

		// configure sensors and simulator
		Simulator& sim = Simulator::getInstance(config, server_ip, server_port);
		if (!sim.configure()) {
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
