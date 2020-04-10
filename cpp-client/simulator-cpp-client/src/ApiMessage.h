#pragma once
#include <string>
#include <cstring>
#include <deque>
#include <sstream>      // std::ostringstream
#include <stdint.h>

#include "Buffer.h"
#include "JsonHelpers.h"


#define CONTROL_HEADER			0x6d6f6e6f
#define RESPONSE_HEADER			0x6f6e6f6d


#include <boost/asio.hpp>

using boost::asio::ip::tcp;


class ApiMessage
{
public:
	enum { header_length = 8 };

	ApiMessage()
		: reference(0),
		success(false)
	{
	}

	ApiMessage(long in_reference, std::string in_type, bool in_success = false, nlohmann::json msg = NULL)
		: reference(in_reference),
		type(in_type),
		success(in_success),
		message(msg)
	{
	}


	ApiMessage(const ApiMessage& other)
		: reference(other.reference),
		type(other.type),
		success(other.success),
		message(other.message)
	{
	}

	std::string uint32_to_string(std::uint32_t value) {
		std::ostringstream os;
		os << value;
		return os.str();
	}
	 
	void read(tcp::socket& socket )
	{
		recvBuffer.resize(header_length);
		boost::asio::read(socket, boost::asio::buffer(recvBuffer.data(), recvBuffer.available()));
		int32_t magic = recvBuffer.readInt();
		int32_t size = recvBuffer.readInt();
		int32_t payloadSize = size - header_length;
		//std::cout<< "payloadSize = " << payloadSize << std::endl;
		if (magic == RESPONSE_HEADER)
		{
			recvBuffer.grow(payloadSize);
			boost::asio::read(socket, boost::asio::buffer(recvBuffer.data(), recvBuffer.available()));
			nlohmann::json j = nlohmann::json::parse(recvBuffer.as_string());
			success = deserialize(j);
		}
	}

	template<typename ReadHandler>
	void asyncRead(tcp::socket& socket, ReadHandler handler)
	{
		recvBuffer.resize(header_length);
		boost::asio::async_read(socket,
			boost::asio::buffer(recvBuffer.data(), recvBuffer.available()),
			[this, &socket, handler](std::error_code ec, std::size_t length)
			{
				if (ec)
				{
					ApiMessage error = make_error(ec.message());
					handler(ec, error);
				}
				else
				{
					int32_t magic = recvBuffer.readInt();
					int32_t size = recvBuffer.readInt();
					int32_t payloadSize = size - header_length;

					if (magic == RESPONSE_HEADER)
					{
						recvBuffer.grow(payloadSize);
						boost::asio::async_read(socket,
							boost::asio::buffer(recvBuffer.data(), recvBuffer.available()),
							[this, handler](std::error_code ec, std::size_t length)
							{
								if (!ec)
								{
									recvBuffer.reset();
									recvBuffer.skip(header_length);
									try
									{
										nlohmann::json j = nlohmann::json::parse(recvBuffer.as_string());
										success = deserialize(j);
									}
									catch (...)
									{
										success = false;
									}
								}

								if (success)
									handler(ec, *this);
								else
									handler(std::make_error_code(std::errc::bad_message), *this);
							});
					}
					else
					{
						ApiMessage error = make_error("Incorrect CONTROL_HEADER sent from server");
						handler(std::make_error_code(std::errc::bad_message), error);
					}
				}
			});
	}

	template<typename WriteHandler>
	void asyncWrite(tcp::socket& socket, WriteHandler handler)
	{
		try {
			std::string data = serialize().dump();
			//std::cout << "ApiMessage::asyncWrite: " << data << std::endl;

			uint32_t length = header_length + data.size();
			sendBuffer.resize(length);
			sendBuffer.writeInt(CONTROL_HEADER);
			sendBuffer.writeInt(length);
			sendBuffer.write((uint8_t*)data.c_str(), data.size());
			sendBuffer.reset();
			boost::asio::async_write(socket,
				boost::asio::buffer(sendBuffer.data(), sendBuffer.available()),
				[this, handler](std::error_code ec, std::size_t length)
				{
					handler(ec, *this);
				});
		}
		catch (std::exception& e) {
			std::cout << "exception during send: " << e.what() << std::endl;
		}
	}

	void write(tcp::socket& socket)
	{
		try {
			//std::cout << "write tcp socket" << std::endl;
			std::string data = serialize().dump();
			//std::cout << "ApiMessage::asyncWrite: " << data << std::endl;

			uint32_t length = header_length + data.size();
			sendBuffer.resize(length);
			sendBuffer.writeInt(CONTROL_HEADER);
			sendBuffer.writeInt(length);
			sendBuffer.write((uint8_t*)data.c_str(), data.size());
			sendBuffer.reset();
			boost::asio::write(socket,
				boost::asio::buffer(sendBuffer.data(), sendBuffer.available()));
		}
		catch (std::exception& e) {
			std::cout << "exception during send: " << e.what() << std::endl;
		}
	}
	
	ApiMessage make_error(std::string error);

	//==================  NetworkMessage Class ====================
	//NetworkMessage Type refers to class that handles message
	const std::string& get_message_type() { return type; }
	void set_message_type(const std::string cls) { type = cls; }

	const bool get_success() { return success; }
	void set_success(const bool val) { success = val; }

	//NetworkMessage Payload
	const nlohmann::json& get_message() { return message; }
	void set_message(const nlohmann::json& in_message) { message = in_message; }

	const long get_reference() { return reference; }
	void set_reference(long in_reference) { reference = in_reference; }

	bool is_valid() { return type.length() > 0; }
	std::string str();

	static std::string from_class(std::string messageClass);

	ApiMessage& operator=(const ApiMessage m) {
		this->reference = m.reference;
		this->type = m.type;
		this->success = m.success;
		this->message = m.message;
		return *this;
	}

	nlohmann::json serialize() {
		nlohmann::json j = nlohmann::json{
			{ "reference", reference },
			{ "type", type },
			{ "success", success },
			{ "message", message }
		};
		return j;
	}

	bool deserialize(nlohmann::json j) {
		reference = json_get_value<long>(j, "reference", 0);
		type = json_get_value<std::string>(j, "type", "");
		success = json_get_value<bool>(j, "success", true);
		message = json_get_value<nlohmann::json>(j, "message", {});
		return type.length() && success;
	}

	bool operator==(const std::string& rhs) {
		return type.compare(rhs) == 0;
	}

	bool operator==(const char* rhs) {
		return type.compare(rhs) == 0;
	}
	//friend bool operator==(const ApiMessage& lhs, const std::string& rhs);
	//friend bool operator==(const std::string& lhs, const ApiMessage& rhs);
	//friend bool operator==(const ApiMessage& lhs, const char* rhs);
	//friend bool operator==(const char* lhs, const ApiMessage& rhs);

private:
	long reference;
	std::string type;
	bool success;
	nlohmann::json message;
	ByteBuffer sendBuffer;
	ByteBuffer recvBuffer;
};

//----------------------------------------------------------------------

typedef std::deque<ApiMessage> api_message_queue;

//----------------------------------------------------------------------

class ErrorMessage : public ApiMessage
{

public:
	ErrorMessage(const ApiMessage& incomingMessage, std::string newError)
		: ApiMessage(incomingMessage)
	{
		set_success(false);
		set_message(newError);
	}
};