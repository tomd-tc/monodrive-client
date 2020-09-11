// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once

#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include "Sensor.h"
#include "Configuration.h"
#include "command_config.h"
#include "ApiMessage.h"


class Simulator 
{
	
public:
	
	~Simulator();

	// Lazy initialization
	static Simulator& getInstance(const Configuration& config);
	static Simulator& getInstance(const std::string& serverIp, const short& serverPort);
	static Simulator& getInstance(const Configuration& config, const std::string& serverIp, const short& serverPort);

	// Cleanup
	static bool deleteInstance(const Configuration& config, const std::string& serverIp, const short& serverPort);
	static void clearInstances();
	
	bool connect();
	bool configure();
	void disconnect();
	void stop();
	bool sendCommand(ApiMessage message, nlohmann::json* response=nullptr);
	bool sendCommandAsync(ApiMessage message, nlohmann::json* response=nullptr);
	bool step(int stepIndex, int numSteps);
	std::thread stepThread(int stepIndex, int numSteps) {
		return std::thread(&Simulator::step, this, stepIndex, numSteps);
	}
	bool stateStepSampleAll(std::vector<std::shared_ptr<Sensor>>& sensors, const nlohmann::json& state);
	void stepSampleAll(std::vector<std::shared_ptr<Sensor>>& sensors, int stepIndex, int numSteps);

	bool stateStepAll(std::vector<std::shared_ptr<Sensor>>& sensors, const nlohmann::json& state);
	bool sampleInProgress(std::vector<std::shared_ptr<Sensor>>& sensors);

	// triggers every sensor on the server to send it's data frame
	// the sensors in the list will go into a read state
	// this should only be called when the sensors in the list count for all the sensors on the server
	bool sampleAll(std::vector<std::shared_ptr<Sensor>>& sensors);
	// samples sensors in the list, if any are not connected returns an error
	bool sampleSensorList(std::vector<std::shared_ptr<Sensor>>& sensors);
	bool sendControl(float forward, float right, float brake, int mode);

	static std::map<const std::string, Simulator*> simMap;

	const std::string& getServerIp() const{return serverIp;}
	const short& getServerPort() const{return serverPort;}
private:
	Simulator(const Configuration& config);
	Simulator(const std::string& serverIp, const short& serverPort);
	Simulator(const Configuration& config, const std::string& serverIp, const short& serverPort);
	Simulator(const Simulator&)= delete;
	void waitForSamples(const std::vector<std::shared_ptr<Sensor>>& sensors);
  	Simulator& operator=(const Simulator&)= delete;

	boost::asio::io_service ioService;
	boost::asio::ip::tcp::socket controlSocket{ioService};
	Configuration config;
	std::string serverIp;
	short serverPort;
	std::atomic<bool> lastSendCommand{false};
};
