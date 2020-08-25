// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#include "Sensor.h"

#include <iostream>
#include <thread>
#include <string>
#include <memory>

#include "Simulator.h"
#include "Stopwatch.h"

Sensor::Sensor(std::unique_ptr<SensorBaseConfig> sensorConfig,
               bool parseBinaryData)
    : config(std::move(sensorConfig)), parseBinaryData(parseBinaryData) {
  name = std::string(config->type) + std::string("_") +
         std::to_string(config->listen_port);
  // ViewportCamera doesn't need a connection
  if (config->listen_port != 0) {
    listener = new Connection(config->server_ip, config->listen_port);
  }
  if (parseBinaryData) {
    frame = config->DataFrameFactory();
  } else {
    frame = new BinaryDataFrame();
  }
  sampleCallback = [](DataFrame* frame) { return; };
}

Sensor::~Sensor()
{
	stopListening();
	delete listener;
	if(frame != nullptr)
	{
		delete frame;
	}
}

bool Sensor::configure()
{
	std::cout << "Configure " << name << std::endl;
	if (!sendConfigure())
	{
		return false;
	}
	if (config->ros.publish_to_ros)
	{
		return true;
	}
	if (!startListening())
	{
		// If streaming wasn't enabled, the configuration is still successful
		return !config->enable_streaming;
	}
	return startSampleLoop();
}
bool Sensor::sendConfigure()
{
	using namespace std;
	Simulator& sim = Simulator::getInstance(config->server_ip, config->server_port);
	nlohmann::json msg = config->dump();
	std::cout << msg << std::endl;
	return sim.sendCommand(ApiMessage(1001, REPLAY_ConfigureSensorsCommand_ID, true, msg));
}

bool Sensor::startListening()
{
	bool success = false;
	if (config->enable_streaming)
	{
		success = listener->connect();
	}
	return success;	
}

bool Sensor::stopListening()
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
	if (sampleThread.joinable())
	{
		sampleThread.join();
	}
	return true;
}

bool Sensor::startSampleLoop()
{
	sampleThread = std::thread([this]() {
		while (bContinue) {
			if (config->listen_port == 0) {
				return false;
			}
			recvBuffer.resize(header_length);
			if (listener->socket.is_open()) {
				try {
					listener->readSensorPacket(recvBuffer);
				}
				catch (const std::exception& e) {
					std::cout << e.what() << std::endl;
					return false;
				}
				parse();
				if (frame->parse_complete()) {
					if (sampleCallback) {
						sampleCallback(frame);
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
	{
		return false;
	}
	frame->parse_header(recvBuffer);
	frame->parse(recvBuffer);
	return true;
}