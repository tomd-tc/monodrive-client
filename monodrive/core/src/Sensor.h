// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once

#include <condition_variable>
#include <string>
#include <thread>
#include <exception>

#include "json.hpp"

#include "Buffer.h"
#include "ApiMessage.h"
#include "SimulatorCommands.h"
#include "sensor_config.h"
#include "DataFrame.h"
#include <chrono>

#define CONTROL_HEADER			0x6d6f6e6f
#define RESPONSE_HEADER			0x6f6e6f6d

class Connection {
public:
	Connection() {};
	~Connection()
	{
		close();
	}
	Connection(std::string& ip, int port)
	{
		auto ipaddress = boost::asio::ip::address::from_string(ip);
		serverEndpoint = boost::asio::ip::tcp::endpoint(ipaddress, port);
	}

	bool readSensorPacket(ByteBuffer& buffer)
	{
		auto headerLength = DATA_FRAME_HEADER_SIZE;
		boost::asio::read(socket, boost::asio::buffer(buffer.data(), buffer.available()));
		auto packetSize = buffer.readInt();
		buffer.readInt();  // time
		buffer.readInt();  // gameTime
		buffer.readInt();  // sampleCount
		auto payloadSize = packetSize - headerLength;
		if (payloadSize > 0)
		{
			buffer.grow(payloadSize);
			boost::asio::read(socket, boost::asio::buffer(buffer.data(), buffer.available()));
		}
		else
		{
			std::cout << "NO DATA AVAILABLE" << std::endl;
		}
		buffer.reset(0);
		return true;
	}

	bool connect()
	{
		std::cout << "Connecting sensor to: " << serverEndpoint << std::endl;
		try
		{
			socket.connect(serverEndpoint);
			if (socket.is_open()){
				std::cout << "Sensor Connected" << std::endl;
			}
			else{
				std::cout << "Sensor Connection FAILED" << std::endl;
			}
			
		}
		catch (std::exception& e)
		{
			std::cout << e.what() << std::endl;
			return false;
		}
		return true;
	}

	inline void close()
	{
		if (socket.is_open())
		{
			socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
			socket.close();
		}
	}
	friend class Sensor;
private:
	boost::asio::ip::tcp::endpoint serverEndpoint;
	boost::asio::io_service ioService;
	boost::asio::ip::tcp::socket socket{ioService};
};

class Sensor  
{
public:
	//Constructors
	~Sensor();
	Sensor(std::unique_ptr<SensorBaseConfig> sensorConfig);
	Sensor(const Sensor& ) = delete;
	Sensor& operator=(const Sensor& ) = delete;

	bool parse();
	bool configure();
	bool sendConfigure();
	bool startListening();
	bool stopListening();
	bool startSampleLoop();

	bool bContinue = true;
	std::thread sampleThread;
	ByteBuffer recvBuffer;
	std::unique_ptr<SensorBaseConfig> config = nullptr;
	DataFrame* frame = nullptr;
	std::atomic<bool> sampleInProgress{false};
	std::function<void(DataFrame*)> sampleCallback;
	
private:
	Connection* listener = nullptr;
	std::string name;
	enum { header_length = DATA_FRAME_HEADER_SIZE };
};

