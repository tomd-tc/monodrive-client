// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
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

std::map<const std::string, Simulator*> Simulator::simMap;

Simulator::Simulator(const Configuration& config)
	: config(config)
{
	serverIp = config.simulator.at("server_ip").get<std::string>();
	serverPort = config.simulator.at("server_port").get<short>();
}

Simulator::Simulator(const Configuration& config, const std::string& serverIp, const short& serverPort)
	: config(config), serverIp(serverIp), serverPort(serverPort)
{}

Simulator::Simulator(const std::string& serverIp, const short& serverPort)
	: serverIp(serverIp), serverPort(serverPort)
{}

Simulator::~Simulator()
{
	stop();
}

Simulator& Simulator::getInstance(const std::string& serverIp, const short& serverPort)
{
	std::lock_guard<std::mutex> simLock(_mutex);
	const std::string simKey = serverIp + ":" + std::to_string(serverPort);
	if (simMap[simKey] == nullptr) {
		simMap[simKey] = new Simulator(serverIp, serverPort);
	}
	simMap[simKey]->connect();
	return *simMap[simKey];
} 


Simulator& Simulator::getInstance(const Configuration& config)
{
		std::string ip = config.simulator.at("server_ip").get<std::string>();
		short port = config.simulator.at("server_port").get<short>();
		return getInstance(config, ip, port);
}

Simulator& Simulator::getInstance(
	const Configuration& config,
	const std::string& serverIp,
	const short& serverPort
)
{
	std::lock_guard<std::mutex> simLock(_mutex);
	const std::string simKey = serverIp + ":" + std::to_string(serverPort);
	if (simMap[simKey] == nullptr) {
		simMap[simKey] = new Simulator(config, serverIp, serverPort);
		std::cout << "created new simulator:" << simKey << std::endl;
	}
	simMap[simKey]->connect();
	return *simMap[simKey];
}

bool Simulator::deleteInstance(
	const Configuration& config,
	const std::string& serverIp,
	const short& serverPort
)
{
	std::lock_guard<std::mutex> simLock(_mutex);
	const std::string simKey = serverIp + ":" + std::to_string(serverPort);
	if (simMap[simKey] == nullptr)
	{
		return false;
	}
	delete simMap[simKey];
	simMap.erase(simKey);
	return true;
}

void Simulator::clearInstances()
{
	std::lock_guard<std::mutex> simLock(_mutex);
	for(auto& sim : simMap){
		delete sim.second;
	}
	simMap.clear();
}

bool Simulator::connect()
{
	if(!controlSocket.is_open())
	{
		std::cout << "******Simulator Connect********" << std::endl;
		try{
		const auto ipaddress = boost::asio::ip::address::from_string(serverIp);
		const auto endpoint = boost::asio::ip::tcp::endpoint(ipaddress, serverPort);
		std::cout << endpoint << std::endl;
		controlSocket.connect(endpoint);
		}
		catch (const std::exception& e){
			std::cerr << "ERROR! Failed to connect to server. Is it running?" << std::endl;
			std::cerr << e.what() << std::endl;
			return false;
		}
	}
	return true;
}

void Simulator::stop()
{
	if (controlSocket.is_open())
	{
		controlSocket.close();
	}
}

bool Simulator::configure()
{
	using namespace std;
	if(!connect())
		return false;

	if(config.simulator.empty())
	{
		std::cout << "WARNING! Skipping Simulator and Scenario Config, no simulator config set." << std::endl;
	}
	else
	{
		std::cout << "Send Simulator Config:   success = ";
		std::cout << sendCommand(ApiMessage(1000, SimulatorConfig_ID, true, config.simulator)) << std::endl;

		int simulation_mode = 0;
		json_get(config.simulator, "simulation_mode", simulation_mode);
		if (!config.scenario.empty())
		{
			if (simulation_mode == 0 or simulation_mode == 3)
			{
				std::cout << "Send Closed Loop Config:    success = ";
				std::cout << sendCommand(ApiMessage(1001, ClosedLoopConfigCommand_ID, true, config.scenario)) << std::endl;
			}
			else
			{
				std::cout << "Send Scenario Config:    success = ";
				std::cout << sendCommand(ApiMessage(1001, REPLAY_ConfigureTrajectoryCommand_ID, true, config.scenario)) << std::endl;
			}
		}
	}


	if (config.weather.empty())
	{
		std::cout << "WARNING! Skipping Weather Config, no weather config set." << std::endl;
	}
	else
	{
		std::cout << "Send Weather Config:     success = ";
		std::cout << sendCommand(ApiMessage(1002, WeatherConfigCommand_ID, true, config.weather)) << std::endl;
	}

	return true;
}

bool Simulator::sendCommand(ApiMessage message, nlohmann::json *response)
{
	message.write(controlSocket);
	ApiMessage res;
	res.read(controlSocket);
	if (response != nullptr)
	{
		*response = res.get_message();
	}
	if (res.get_success())
	{
		return true;
	}
	std::cerr << res.get_message().dump() << std::endl;
	return false;
}

bool Simulator::step(int stepIndex, int numSteps)
{
	nlohmann::json msg{{"amount", numSteps}};
	ApiMessage message(stepIndex, REPLAY_StepSimulationCommand_ID, true, msg);
	return sendCommand(message);
}

bool Simulator::stateStepSampleAll(std::vector<std::shared_ptr<Sensor>>& sensors, const nlohmann::json& state)
{
	ApiMessage message(333, REPLAY_StateStepSimulationCommand_ID, true, state);
	for(auto& sensor : sensors)
    {
		sensor->sampleInProgress.store(true, std::memory_order::memory_order_relaxed);
	}
	bool success = sendCommand(message);
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

void Simulator::stepSampleAll(std::vector<std::shared_ptr<Sensor>>& sensors, int stepIndex, int numSteps)
{
	for(auto& sensor : sensors)
    {
		sensor->sampleInProgress.store(true, std::memory_order::memory_order_relaxed);
	}
	step(stepIndex, numSteps);
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

bool Simulator::sampleAll(std::vector<std::shared_ptr<Sensor>>& sensors)
{
	ApiMessage sampleMessage(999, SampleSensorsCommand_ID, true, {});
	for(auto& sensor : sensors){
		sensor->sampleInProgress.store(true, std::memory_order::memory_order_relaxed);
	}
	if(sendCommand(sampleMessage)){
		waitForSamples(sensors);
	}
	else{
		std::cerr << "ERROR! Failed to sample sensors." << std::endl;
		return false;
	}
	return true;
}

bool Simulator::sampleSensorList(std::vector<std::shared_ptr<Sensor>>& sensors)
{
	std::vector<int> ports;
	for(auto& sensor : sensors){
		ports.push_back(sensor->config->listen_port);
	}
	ApiMessage sampleMessage(999, SampleSensorListCommand_ID, true, {{"ports", ports}});
	for(auto& sensor : sensors){
		sensor->sampleInProgress.store(true, std::memory_order::memory_order_relaxed);
	}
	if(sendCommand(sampleMessage)){
		waitForSamples(sensors);
	}
	else{
		std::cerr << "ERROR! Failed to sample sensors." << std::endl;
		return false;
	}
	return true;
}

void Simulator::waitForSamples(const std::vector<std::shared_ptr<Sensor>>& sensors)
{
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

bool Simulator::sendControl(float forward, float right, float brake, int mode)
{
    nlohmann::json ego_msg;
    ego_msg["forward_amount"] = forward;
    ego_msg["right_amount"] = right;
    ego_msg["brake_amount"] = brake;
    ego_msg["drive_mode"] = mode;
    return sendCommand(ApiMessage(123, EgoControl_ID, true, ego_msg));
}
