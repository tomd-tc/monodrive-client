#include <iostream>
#include <thread>
#include <string>
#include <memory>

#include "Sensor.h"
#include "Simulator.h"
#include "Stopwatch.h"


Sensor::Sensor(std::unique_ptr<SensorBaseConfig> sensor_config) : config(std::move(sensor_config))
{
	name = std::string(config->type) + std::string("_") + std::to_string(config->listen_port);
    //VieportCamera doesn't need a connection 
	if(config->listen_port != 0){
		listener = new Connection(config->server_ip, config->listen_port);
	} 
	frame = config->DataFrameFactory();
}

Sensor::~Sensor()
{
	stop_listening();
	delete listener;
	if(frame != nullptr)
		delete frame;
}

bool Sensor::configure()
{
	std::cout << "Configure " << name << std::endl;
	bool success = send_configure();
	if(!config->ros.publish_to_ros){
		success = start_listening();
	}
	return success;
}
bool Sensor::send_configure()
{
	using namespace std;
	Simulator& sim = Simulator::getInstance(config->server_ip, config->server_port);
	nlohmann::json msg = config->dump();
	std::cout << msg << std::endl;
	// nlohmann::json::parse(dump_json());
	return sim.send_command(ApiMessage(1001, REPLAY_ConfigureSensorsCommand_ID, true, msg));	
}

bool Sensor::start_listening()
{
	bool success = false;
	if(config->listen_port !=0)
	{
		success = listener->open_connection();
	}
	return success;	
}

bool Sensor::stop_listening()
{
	bContinue = false;
	if(listener==nullptr)
		return true;
	if(listener->socket.is_open()){
		listener->socket.close();
	}
	SampleThread.join();
	return true;
}

bool Sensor::StartSampleLoop()
{
	SampleThread = std::thread([this](){
		while(bContinue){
			if(config->listen_port == 0)
				return false;
			recvBuffer.resize(header_length);
			if(listener->socket.is_open()){
				mono::precise_stopwatch watch0;
				// std::cout << "reading data frame..." << std::endl;
				listener->read_sensor_packet(recvBuffer);
				// std::cout << "read success..." << std::endl;
				std::cout << name << " read : " <<  watch0.elapsed_time<unsigned int, std::chrono::milliseconds>() << " (ms)" << std::endl;
				mono::precise_stopwatch watch1;
				parse();
				std::cout << name << " parse: " <<  watch1.elapsed_time<unsigned int, std::chrono::microseconds>() << " (us)" << std::endl;
				if(frame->parse_complete()){
					// std::cout << "PARSE COMPLETE" << std::endl;
					mono::precise_stopwatch watch2;
					sample_callback(frame);
					std::cout << name << " callback: " << watch2.elapsed_time<unsigned int, std::chrono::milliseconds>() << " (ms)" << std::endl;
					sampleInProgress.store(false, std::memory_order::memory_order_relaxed);
				}
			}
			else{
				std::cout << "Sensor Channel is not open" << std::endl;
				return false;
			}
		}
		return true;
	});
	return true;
}

bool Sensor::parse(){
	if(frame == nullptr)
		return false;
	frame->parse(recvBuffer);
	return true;
}