#pragma once

#include "Buffer.h"
#include "JsonHelpers.h"
#include "DataFramePrimitives.h"
#include <algorithm>
#include "UECompatability.h"

class DataFrame{
public:
    virtual void parse(ByteBuffer& buffer) = 0;
    virtual ByteBuffer write() const = 0;
    virtual bool parse_complete(){
        return true;
    } 
	virtual ~DataFrame() {}
};

class MONODRIVECORE_API RadarTargetListFrame : public DataFrame{
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

class MONODRIVECORE_API ImuFrame : public DataFrame{
public:
    virtual void parse(ByteBuffer& buffer) override;
    virtual ByteBuffer write() const override;

    Vec3f angular_velocity;
    Vec3f acceleration;
    uint32_t timer;
    uint16_t checksum;
    int time_of_week;
};

class MONODRIVECORE_API GPSFrame : public DataFrame{
public:
    virtual void parse(ByteBuffer& buffer) override;
    virtual ByteBuffer write() const override;
    uint8_t preamble{0x55};
    uint16_t MSG_POS_LLH{0x020A};
    uint16_t id_hash;
    uint8_t payload_size{34};

    double lattitude;
    double longitude;
    double elevation;

    float world_x;
    float world_y;
    Vec3f forward;
    float yaw;
    float speed;

    uint16_t horizontal_accuracy;
    uint16_t vertical_accuracy;
    uint8_t num_sats_signal;
    uint8_t fixed_mode_status;
    uint16_t crc;
};

class MONODRIVECORE_API StateFrame : public DataFrame{
public:
    virtual void parse(ByteBuffer& buffer) override;
	virtual ByteBuffer write() const override;
    std::vector<VehicleState> vehicles;
    std::vector<ObjectState> objects;
    float game_time;
    int time;
	int sample_count;
};

// for now 8 bit only, todo: add float higher bit rate etc enum
class MONODRIVECORE_API ImageFrame : public DataFrame{
public:
    ImageFrame(int x_res, int y_res, int channels) : channels(channels){
        resolution.x = x_res;
        resolution.y = y_res;
        pixels = new uint8_t[channels * resolution.x * resolution.y];
    }
    ~ImageFrame(){
        delete[] pixels;
    }
    uint8_t* pixels;
    int channels;
    struct Resolution{
        int x;
        int y;
    } resolution;
    int size() const{
        return resolution.x * resolution.y * channels;
    }
    virtual void parse(ByteBuffer& buffer);
    virtual ByteBuffer write() const override;
};


class MONODRIVECORE_API CameraAnnotationFrame : public DataFrame{
public:
	virtual void parse(ByteBuffer& buffer) override;
	virtual ByteBuffer write() const override;
	std::map<std::string, AnnotationFrame2D> annotations;
};

class MONODRIVECORE_API CameraFrame : public DataFrame{
public:
	virtual void parse(ByteBuffer& buffer) override;
	virtual ByteBuffer write() const override;
    CameraFrame(int x_res, int y_res, int channels, bool hasAnnotation) : 
        bHasAnnotation(hasAnnotation),
        currentFrameIndex(0)
    {
        imageFrame = new ImageFrame(x_res, y_res, channels);
        annotationFrame = new CameraAnnotationFrame();
    }
    ~CameraFrame(){
        delete imageFrame;
        delete annotationFrame;
    }
    // for the double send on image then annotation
    virtual bool parse_complete(){
        if(!bHasAnnotation or currentFrameIndex % 2 == 0){
            return true;
        }
        else{
            return false;
        }
    }
    ImageFrame* imageFrame;
    CameraAnnotationFrame* annotationFrame;
    bool bHasAnnotation;
    int currentFrameIndex;
};