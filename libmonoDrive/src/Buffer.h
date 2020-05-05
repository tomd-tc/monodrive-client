#pragma once

#include <stdint.h>
#include <exception>
#include <string>
#include <cstring>
#include "JsonHelpers.h"


class BufferOverrunException : public std::exception {
public:
	BufferOverrunException() {}
	~BufferOverrunException() {}
};

class ByteBuffer {
public:
	ByteBuffer()
		: data_(nullptr),
		length_(0),
		position_(0)
	{}

	ByteBuffer(uint32_t size)
		: length_(size),
		position_(0)
	{
		data_ = new uint8_t[size];
	}

	ByteBuffer(uint32_t size, uint32_t header_offset)
		: length_(size+header_offset),
		position_(header_offset)
	{
		data_ = new uint8_t[length_];
	}

	ByteBuffer(const ByteBuffer& other)
	{
		length_ = other.length_;
		position_ = other.position_;
		data_ = new uint8_t[length_];
		if (length_ > 0) {
			memcpy(data_, other.data_, length_);
		}
	}

	virtual ~ByteBuffer() {
		delete[] data_;
	}
	
	void resize(uint32_t size) {
		delete[] data_;
		data_ = new uint8_t[size];
		length_ = size;
		position_ = 0;
	}

	ByteBuffer& operator+=(uint32_t n) {
		skip(n);
		return *this;
	}

	void* data() const {
		return &data_[position_];
	}

	void grow(uint32_t size) {
		uint8_t* newdata = new uint8_t[length_ + size];
		if (length_ > 0) {
			memcpy(newdata, data_, length_);
			delete[] data_;
		}
		length_ += size;
		data_ = newdata;
	}

	void reset(uint32_t offset = 0) {
		position_ = offset;
	}

	uint32_t size() const { return available(); }
	uint32_t length() const { return length_; }

	uint32_t skip(uint32_t count) {
		position_ += count;
		if (position_ > length_)
			position_ = length_;
		return position_;
	}

	int32_t available() const {
		return length_ - position_;
	}

	int read() {
		if (position_ < length_) {
			return data_[position_++] & 0xff;
		}
		return -1;
	}

	uint8_t readByte() {
		int b = read();
		if (b < 0)
			throw BufferOverrunException();

		return b & 0xff;
	}

	uint16_t readShort() {
		return (readByte() << 8) | readByte();
	}

	uint32_t readInt() {
		return (readShort() << 16) | readShort();
	}

	uint64_t readLong() {
		uint64_t value = readInt();
		return (value << 32) | readInt();
	}

	double readDouble(){
		uint64_t val = readLong();
		union{
			uint64_t input;
			double output;
		} data;
		data.input = val;
		return data.output;
	}

	float readFloat(){
		uint32_t val = readInt();
		union{
			uint32_t input;
			float output;
		} data;
		data.input = val;
		return data.output;
	}

	void write(uint8_t value) {
		if (position_ + 1 >= length_) {
			grow((position_ + 1) - length_);
		}
		data_[position_++] = value;
	}

	void writeShort(int16_t value) {
		write((value >> 8) & 0xff);
		write(value & 0xff);
	}

	void writeShortLE(int16_t value) {
		write(value & 0xff);
		write((value >> 8) & 0xff);
	}

	void writeInt(int32_t value) {
		writeShort((value >> 16) & 0xffff);
		writeShort(value & 0xffff);
	}

	void writeIntLE(int32_t value) {
		writeShortLE(value & 0xffff);
		writeShortLE((value >> 16) & 0xffff);
	}

	void writeLong(int64_t value) {
		writeInt((value >> 32) & 0xffffffff);
		writeInt(value & 0xffffffff);
	}

	void write(uint8_t* buffer, uint32_t length) {
		if (position_ + length > length_) {
			grow((position_ + length) - length_);
		}
		memcpy(&data_[position_], buffer, length);
		position_ += length;
	}

	std::string as_string() const {
		std::string str;
		str.assign((char*)&data_[position_], (std::size_t)(length_ - position_));
		return str;
	}

    inline nlohmann::json BufferToJson() const{
		return nlohmann::json::parse(as_string());
	}
    inline static ByteBuffer JsonToBuffer(const nlohmann::json& frame){
		std::string raw = frame.dump();
		ByteBuffer buffer((uint32_t)raw.size(), 12);
		buffer.write((uint8_t*)raw.c_str(), (uint32_t)raw.size());
		buffer.reset(12);
		std::cout << "buffer dump: |" << buffer.as_string() << "|" << std::endl;
		return buffer;
	}


private:
	uint8_t* data_;
	uint32_t length_;
	uint32_t position_;
};