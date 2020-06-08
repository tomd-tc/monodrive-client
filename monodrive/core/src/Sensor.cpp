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
	sample_callback = [](DataFrame* frame){return;};
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
	if (listener == nullptr)
	{
		return true;
	}
	if (listener->socket.is_open())
	{
		listener->close();
	}
	if (SampleThread.joinable())
	{
		SampleThread.join();
	}
	return true;
}

bool Sensor::StartSampleLoop()
{
	SampleThread = std::thread([this]() {
		while (bContinue) {
			if (config->listen_port == 0)
				return false;
			recvBuffer.resize(header_length);
			if (listener->socket.is_open()) {
				try {
					listener->read_sensor_packet(recvBuffer);
				}
				catch (const std::exception& e) {
					std::cout << e.what() << std::endl;
					return false;
				}
				parse();
				if (frame->parse_complete()) {
					if (sample_callback) {
						sample_callback(frame);
					}
					sampleInProgress.store(false, std::memory_order::memory_order_relaxed);
				}
			}
			else {
				std::cout << "Sensor Channel is not open" << std::endl;
				return false;
			}
		}
		return true;
	});
	return true;
}

bool Sensor::parse()
{
	if(frame == nullptr)
		return false;
	frame->parse_header(recvBuffer);
	frame->parse(recvBuffer);
	return true;
}