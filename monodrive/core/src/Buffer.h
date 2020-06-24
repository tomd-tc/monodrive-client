// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once

#include <stdint.h>
#include <exception>
#include <string>
#include <cstring>
#include <algorithm>
#include <bitset>
#include <limits.h>
#include "JsonHelpers.h"

#define DATA_FRAME_HEADER_SIZE 16

class BufferOverrunException : public std::exception {
public:
	BufferOverrunException() {}
	~BufferOverrunException() {}
};

class ByteBuffer {
public:
	ByteBuffer(size_t size = 0)
		: length_(size),
		position_(0),
		data_(size ? new uint8_t[length_]() : nullptr)
	{
	}

	ByteBuffer(size_t size, size_t offset)
		: length_(size+offset),
		position_(offset),
		data_(size ? new uint8_t[length_]() : nullptr)
	{
	}

	ByteBuffer(const ByteBuffer& other)
		: length_(other.length_),
		position_(other.position_),
		data_(length_ ? new uint8_t[length_]() : nullptr)
	{
		std::copy(other.data_, other.data_+length_, data_);
	}

	friend void swap(ByteBuffer& lhs, ByteBuffer& rhs){
		using std::swap;
		swap(lhs.data_, rhs.data_);
		swap(lhs.length_, rhs.length_);
		swap(lhs.position_, rhs.position_);
	}

	ByteBuffer& operator=(ByteBuffer other){
		swap(*this, other);
		return *this;
	}

	ByteBuffer(ByteBuffer&& other) noexcept
		: ByteBuffer()
	{
		swap(*this, other);
	}

	virtual ~ByteBuffer() {
		delete[] data_;
	}

	size_t position() const {
		return position_;
	}
	
	void resize(size_t size) {
		delete[] data_;
		data_ = new uint8_t[size];
		length_ = size;
		position_ = 0;
	}

	ByteBuffer& operator+=(size_t n) {
		skip(n);
		return *this;
	}

	void* data() const {
		return &data_[position_];
	}

	void grow(size_t size) {
		uint8_t* newdata = new uint8_t[length_ + size];
		if (length_ > 0) {
			memcpy(newdata, data_, length_);
			delete[] data_;
		}
		length_ += size;
		data_ = newdata;
	}

	void reset(size_t offset = 0) {
		position_ = offset;
	}

	size_t size() const { return available(); }
	size_t length() const { return length_; }

	size_t skip(size_t count) {
		position_ += count;
		if (position_ > length_)
			position_ = length_;
		return position_;
	}

	size_t available() const {
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

	void writeDouble(double value){
		union{
			double input;
			uint64_t output;
		} data;
		data.input = value;
		std::bitset<sizeof(double)*CHAR_BIT> bits(data.output);
		writeLong((uint64_t)bits.to_ullong());
	}

	void writeFloat(float value){
		union{
			float input;
			uint32_t output;
		} data;
		data.input = value;
		std::bitset<sizeof(float)*CHAR_BIT> bits(data.output);
		writeInt((uint32_t)bits.to_ulong());
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

	void write(uint8_t* buffer, size_t length) {
		if (position_ + length > length_) {
			grow((position_ + length) - length_);
		}
		memcpy(&data_[position_], buffer, length);
		position_ += length;
	}

	inline std::string as_string() const {
		if (size() == 0)
			return std::string("");
		return std::string((char*)data(), size());
	}

	inline nlohmann::json BufferToJson() const {
		return nlohmann::json::parse(as_string());
	}

	inline static ByteBuffer JsonToBuffer(const nlohmann::json& frame){
		std::string raw = frame.dump();
		ByteBuffer buffer(raw.size(), DATA_FRAME_HEADER_SIZE);
		buffer.write((uint8_t*)raw.c_str(), raw.size());
		buffer.reset(DATA_FRAME_HEADER_SIZE);
		
		return buffer;
	}


private:
	size_t length_;
	size_t position_;
	uint8_t* data_;
};
