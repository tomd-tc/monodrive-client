#pragma once

#include "Buffer.h"
#include "JsonHelpers.h"
#include "DataFramePrimitives.h"
#include <algorithm>

class DataFrame{
public:
    virtual void parse(ByteBuffer& buffer) = 0;
    virtual ByteBuffer write() const = 0;
	virtual ~DataFrame() {}
    static ByteBuffer JsonToBuffer(const nlohmann::json& frame);
    static nlohmann::json BufferToJson(const ByteBuffer& buffer);
    // int read_header(ByteBuffer& buffer);
    // uint32_t time;
    // float game_time;
};

// for now 8 bit only, todo: add float higher bit rate etc enum
class ImageFrame : public DataFrame{
public:
    ImageFrame(int x_res, int y_res, int channels) : channels(channels){
        resolution.x = x_res;
        resolution.y = y_res;
        pixels = new uint8_t[channels * resolution.x * resolution.y];
    }
    ~ImageFrame(){
        delete pixels;
    }
    uint8_t* pixels;
    int channels;
    struct Resolution{
        int x;
        int y;
    } resolution;
    int size(){
        return resolution.x * resolution.y * channels;
    }
    virtual void parse(ByteBuffer& buffer){
        memcpy(pixels, buffer.data(), buffer.size());
    }
    virtual ByteBuffer write() const override{
        // todo
        return ByteBuffer();
    }
};

class RadarTargetListFrame : public DataFrame{
public:
    virtual void parse(ByteBuffer& buffer) override;
    virtual ByteBuffer write() const override;
public:
    struct Target{
        std::vector<std::string> target_ids;
        float range{-1}; // m
        float aoa{0}; // degrees
        float velocity{0};
        float rcs{0}; 
    };
    std::vector<Target> targets;
    std::vector<Target> gt_targets;
protected:
    void parse_target_list(const nlohmann::json& target_list);
    void parse_gt_target_list(const nlohmann::json& target_list);
    nlohmann::json write_target_list() const;
    nlohmann::json write_gt_target_list() const;
};

class ImuFrame : public DataFrame{
public:
    virtual void parse(ByteBuffer& buffer) override;
    virtual ByteBuffer write() const override {
        // todo
        return ByteBuffer();
    }

    float acc_x, acc_y, acc_z;
    float ang_x, ang_y, ang_z;
    uint32_t timer;
    uint16_t checksum;
    int time_of_week;
};

class GPSFrame : public DataFrame{
public:
    virtual void parse(ByteBuffer& buffer) override;
    virtual ByteBuffer write() const override {
        // todo
        return ByteBuffer();
    }
    // uint8_t preamble;
    // uint16_t MSG_POS_LLH;
    uint16_t id_hash;
    // uint8_t payload_size;

    double lattitude;
    double longitude;
    double elevation;

    float world_x;
    float world_y;
    float forward_x;
    float forward_y;
    float forward_z;
    float yaw;
    float speed;

    uint16_t horizontal_accuracy;
    uint16_t vertical_accuracy;
    uint8_t num_sats_signal;
    uint8_t fixed_mode_status;
    uint16_t crc;
};

class StateFrame : public DataFrame{
public:
    virtual void parse(ByteBuffer& buffer) override;
    virtual ByteBuffer write() const override {
        // todo
        return ByteBuffer();
    }
    std::vector<VehicleState> vehicles;
    float game_time;
    int time;
};

class CameraAnnotationFrame : public DataFrame{
public:
	virtual void parse(ByteBuffer& buffer) override;
	virtual ByteBuffer write() const override;
	std::map<std::string, AnnotationFrame2D> annotations;
};