#include <iostream>
#include <thread>
#include <string>
#include <memory>


#include "Sensor.h"
#include "Simulator.h"


Sensor::Sensor(SensorBaseConfig& sensor_config) : config(&sensor_config)
{
    //VieportCamera doesn't need a connection 
	if(config->listen_port != 0){
		listener = new Connection(config->server_ip, config->listen_port);
	} 

	name = std::string(config->type) + std::string("_") + std::to_string(config->listen_port);
}

Sensor::~Sensor()
{
	stop_listening();
	delete listener;
}

std::string Sensor::dump_json()
{
	return config->dump_json();
}

bool Sensor::configure()
{
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
	nlohmann::json msg = nlohmann::json::parse(dump_json());
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
	if(listener==nullptr)
		return true;
	if(listener->socket.is_open()){
		listener->socket.close();
	}
	return true;
}

void Sensor::sample()
{
	if(config->listen_port == 0)
		return;
	recvBuffer.resize(header_length);
	if(listener->socket.is_open()){
    	listener->read_sensor_packet(recvBuffer);
	}
	else
	{
		std::cout << "Sensor Channel is not open" << std::endl;
	}
}
