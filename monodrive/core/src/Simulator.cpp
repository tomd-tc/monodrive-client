#include <future>
#include <thread>

#include <fstream>
#include <iostream>
#include <string>
#include <exception>

#include "Simulator.h"
#include "ApiMessage.h"
#include "SimulatorCommands.h"

static std::mutex _mutex;

std::map<const std::string, Simulator*> Simulator::sim_map;

Simulator::Simulator(const Configuration& inConfig)
	: config(inConfig)
{
	server_ip = config.simulator.at("server_ip").get<std::string>();
	server_port = config.simulator.at("server_port").get<short>();
}

Simulator::Simulator(const Configuration& inConfig, const std::string& inServer_ip, const short& inServer_port)
	: config(inConfig), server_ip(inServer_ip), server_port(inServer_port)
{

}

Simulator::~Simulator()
{
	if (controlSocket.is_open())
		{
			controlSocket.close();
		}
}

Simulator& Simulator::getInstance(const std::string& inServer_ip, const short& inServer_port)
{
		std::lock_guard<std::mutex> simLock(_mutex);
		const std::string sim_key = inServer_ip +":" + std::to_string(inServer_port);
		return *sim_map[sim_key];
} 


Simulator& Simulator::getInstance(const Configuration& inConfig) 
{
		std::lock_guard<std::mutex> simLock(_mutex);
		std::string ip_address = inConfig.simulator.at("server_ip").get<std::string>();
		short port = inConfig.simulator.at("server_port").get<short>();
		return getInstance(inConfig, ip_address, port);
}

Simulator& Simulator::getInstance(const Configuration& inConfig, 
											  const std::string& inServer_ip, 
											  const short& inServer_port) 
 	{
		std::lock_guard<std::mutex> simLock(_mutex);
		const std::string sim_key = inServer_ip +":" + std::to_string(inServer_port);
		if (sim_map[sim_key] == nullptr) {
			sim_map[sim_key] = new Simulator(inConfig, inServer_ip, inServer_port);
			std::cout<<"created new simulator:"<< sim_key << std::endl;
		}
		return *sim_map[sim_key];
	}

void Simulator::connect()
{
	std::cout << "******Simulator Connect********" << std::endl;
	const auto ipaddress = boost::asio::ip::address::from_string(server_ip);
	const auto endpoint = boost::asio::ip::tcp::endpoint(ipaddress, server_port);
	std::cout << endpoint << std::endl;
	controlSocket.connect(endpoint);
}

bool Simulator::configure()
{
	using namespace std;
	if(!controlSocket.is_open())
	{
		try{
			connect();
		}
		catch (const std::exception& e){
			std::cout << "Failed to connect to server. Is it running?" << std::endl;
			std::cerr << e.what() << std::endl;
			return false;
		}
	}

	std::cout << "Send Simulator Config:   success = ";
	std::cout << send_command(ApiMessage(1000, SimulatorConfig_ID, true, config.simulator)) << std::endl;

	int simulation_mode = 0;
	json_get(config.simulator, "simulation_mode", simulation_mode);
	if(simulation_mode == 0 or simulation_mode == 3) {
		std::cout << "Send Closed Loop Config:    success = ";
		std::cout << send_command(ApiMessage(1001, ClosedLoopConfigCommand_ID, true, config.scenario)) << std::endl;
	} else {
		std::cout << "Send Scenario Config:    success = ";
		std::cout << send_command(ApiMessage(1001, REPLAY_ConfigureTrajectoryCommand_ID, true, config.scenario)) << std::endl;
	}

	std::cout << "Send Weather Config:     success = ";
	std::cout << send_command(ApiMessage(1002, WeatherConfigCommand_ID, true, config.weather)) << std::endl;	
	return true;
}

bool Simulator::send_command(ApiMessage msg, nlohmann::json* resp_message)
{
  msg.write(controlSocket);
  ApiMessage response;
  response.read(controlSocket);
	if(resp_message != nullptr) {
		*resp_message = response.get_message();
	}
  if (response.get_success()) {
    return true;
	}
	return false;
}

bool Simulator::step(int step_idx, int nsteps)
{
	nlohmann::json msg{{"amount", nsteps}};
	ApiMessage step_message(step_idx, REPLAY_StepSimulationCommand_ID, true, msg);
	return send_command(step_message);
}

bool Simulator::state_step_sample_all(std::vector<std::shared_ptr<Sensor>>& sensors, const nlohmann::json& state)
{
	ApiMessage step_message(333, REPLAY_StateStepSimulationCommand_ID, true, state);	
	for(auto& sensor : sensors)
    {
		sensor->sampleInProgress.store(true, std::memory_order::memory_order_relaxed);
	}
	bool success = send_command(step_message);
	if(!success)
		return success;
	bool samplingInProgress = true;
	do{
		samplingInProgress = false;
		for(auto& sensor : sensors){
			if(sensor->sampleInProgress.load(std::memory_order::memory_order_relaxed)){
				samplingInProgress = true;
				break;
			}
		}
	} while(samplingInProgress);
	return success;
}

void Simulator::step_sample_all(std::vector<std::shared_ptr<Sensor>>& sensors, int step_idx, int nsteps)
{
	for(auto& sensor : sensors)
    {
		sensor->sampleInProgress.store(true, std::memory_order::memory_order_relaxed);
	}
	step(step_idx, nsteps);
	bool samplingInProgress = true;
	do{
		samplingInProgress = false;
		for(auto& sensor : sensors){
			if(sensor->sampleInProgress.load(std::memory_order::memory_order_relaxed)){
				samplingInProgress = true;
				break;
			}
		}
	} while(samplingInProgress);
}

void Simulator::sample_all(std::vector<std::shared_ptr<Sensor>>& sensors)
{
	ApiMessage sampleMessage(999, SampleSensorsCommand_ID, true, {});
	for(auto& sensor : sensors){
		sensor->sampleInProgress.store(true, std::memory_order::memory_order_relaxed);
	}
	send_command(sampleMessage);
	bool samplingInProgress = true;
	do{
		samplingInProgress = false;
		for(auto& sensor : sensors){
			if(sensor->sampleInProgress.load(std::memory_order::memory_order_relaxed)){
				samplingInProgress = true;
				break;
			}
		}
	} while(samplingInProgress);
}
