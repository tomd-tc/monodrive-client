#pragma once

#include <condition_variable>
#include <string>
#include <thread>
#include <exception>

#include "boost/asio.hpp"
#include "json.hpp"

#include "Buffer.h"
#include "ApiMessage.h"
#include "SimulatorCommands.h"
#include "sensor_config.h"

#define CONTROL_HEADER			0x6d6f6e6f
#define RESPONSE_HEADER			0x6f6e6f6d

//enum class SensorType {Lidar, Radar, Camera, IMU, GPS, Collision};

//template<typename SensorT>

class Connection  {
public:
	Connection(){};
	~Connection(){
		socket.close();
	}
	Connection(std::string& ip, int port){
		auto ipaddress = boost::asio::ip::address::from_string(ip);
		server_endpoint = boost::asio::ip::tcp::endpoint(ipaddress, port);
	}
    //Connection(boost::asio::io_service& io_service) : socket(io_service) {}
	bool read_sensor_packet(ByteBuffer& buffer)
	{
		auto header_length = 12;
		boost::asio::read(socket, boost::asio::buffer(buffer.data(), buffer.available()));
		auto packet_size = buffer.readInt();
		auto time = buffer.readInt();
		auto gameTime = buffer.readInt();
		auto payloadSize = packet_size - header_length;

		if (payloadSize > 0)
		{
			// std::cout<< "received:" << std::to_string(payloadSize) << " bytes" << std::endl;
			buffer.grow(payloadSize);
			boost::asio::read(socket, boost::asio::buffer(buffer.data(), buffer.available()));
		}
		else
		{
			std::cout << "NO DATA AVAILABLE" << std::endl;
		}
		return true;
	}

	bool open_connection()
	{
		std::cout << "Connectiong sensor to: " << server_endpoint << std::endl;
		try
		{
			socket.connect(server_endpoint);
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
	friend class Sensor;
private:
	boost::asio::ip::tcp::endpoint server_endpoint;
	boost::asio::io_service io_service;
    boost::asio::ip::tcp::socket socket{io_service};
};

class Sensor  
{
public:
	//Constructors
	Sensor(){};
	~Sensor();
	Sensor(SensorBaseConfig& sensor_config);

	void sample();
	std::string dump_json();
	bool configure();
	bool send_configure();
	bool start_listening();
	
	ByteBuffer recvBuffer;
	SensorBaseConfig* config = nullptr;
	
private:
	std::string name;
	enum { header_length = 12 };
	Connection* listener = nullptr;
};

