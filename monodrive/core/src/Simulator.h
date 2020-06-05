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
	static Simulator& getInstance(const Configuration& inConfig);
	static Simulator& getInstance(const std::string& inServer_ip, const short& inServer_port);
	static Simulator& getInstance(const Configuration& inConfig, const std::string& inServer_ip, const short& inServer_port);

	// Cleanup
	static bool deleteInstance(const Configuration& inConfig, const std::string& inServer_ip, const short& inServer_port);
	static void clearInstances();
	
	void connect();
	bool configure();
	void disconnect();
	void stop();
	bool send_command(ApiMessage msg, nlohmann::json* response_message=nullptr);
	bool step(int step_idx, int nsteps);
	std::thread stepThread(int step_idx, int nsteps) {
		return std::thread(&Simulator::step, this, step_idx, nsteps);
	}
	bool state_step_sample_all(std::vector<std::shared_ptr<Sensor>>& sensors, const nlohmann::json& state);
	void step_sample_all(std::vector<std::shared_ptr<Sensor>>& sensors, int step_idx, int nsteps);
	void sample_all(std::vector<std::shared_ptr<Sensor>>& sensors);
	bool sendControl(float forward, float right, float brake, int mode);

	static std::map<const std::string, Simulator*> sim_map;

	const std::string& getServerIp() const{return server_ip;}
	const short& getServerPort() const{return server_port;}
private:
	Simulator(const Configuration& inConfig);
	Simulator(const Configuration& inConfig, const std::string& inServer_ip, const short& inServer_port);
	Simulator(const Simulator&)= delete;
  	Simulator& operator=(const Simulator&)= delete;

	boost::asio::io_service io_service;
	boost::asio::ip::tcp::socket controlSocket{io_service};
	Configuration config;
	std::string server_ip;
	short server_port;
};