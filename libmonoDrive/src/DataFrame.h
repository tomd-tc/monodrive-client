#pragma once

#include "Buffer.h"
#include "JsonHelpers.h"
#include "DataFramePrimitives.h"
#include <algorithm>
#include "UECompatability.h"
#include <complex>

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
    std::vector<RadarTarget> targets;
    std::vector<RadarTarget> gt_targets;
};

class MONODRIVECORE_API RadarCubeFrame : public DataFrame{
public:
    RadarCubeFrame(int numSweeps, int numSamplesPerSweep, int numElements){
        radar_cube.resize(numSweeps*numSamplesPerSweep*numElements);
    }
    virtual void parse(ByteBuffer& buffer) override;
    virtual ByteBuffer write() const override;
    inline size_t size() const{
        return radar_cube.size();
    }
    int numSweeps;
    int numSamplesPerSweep;
    int numElements;
    std::vector<std::complex<float>> radar_cube;
};

class MONODRIVECORE_API RadarFrame : public DataFrame{
public:
    RadarFrame(bool send_radar_cube, int numSweeps, int numSamplesPerSweep, int numElements) 
    : bSendRadarCube(send_radar_cube),
    currentFrameIndex(0) 
    {
        radarTargetListFrame = new RadarTargetListFrame();
        radarCubeFrame = new RadarCubeFrame(numSweeps, numSamplesPerSweep, numElements); 
    }
    ~RadarFrame(){
        delete radarTargetListFrame;
    }
    virtual void parse(ByteBuffer& buffer) override;
    virtual ByteBuffer write() const override;
public:
    RadarTargetListFrame* radarTargetListFrame;
    RadarCubeFrame* radarCubeFrame;
    bool bSendRadarCube;
    int currentFrameIndex;
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
    ImageFrame(int x_res, int y_res, int channels) 
        : channels(channels)
    {
        resolution.x = x_res;
        resolution.y = y_res;
        pixels = new uint8_t[channels * resolution.x * resolution.y];
    }
    ~ImageFrame(){
        delete[] pixels;
    }
    int channels;
    uint8_t* pixels;
    struct Resolution{
        int x;
        int y;
    } resolution;
    inline int size() const{
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