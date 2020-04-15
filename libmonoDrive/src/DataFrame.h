#pragma once

#include "Buffer.h"
#include "JsonHelpers.h"
#include <algorithm>
#include "Configuration.h"

class DataFrame{
public:
    virtual void parse(ByteBuffer& buffer) = 0;
    virtual ByteBuffer write() const = 0;
    static ByteBuffer JsonToBuffer(const nlohmann::json& frame);
    static nlohmann::json BufferToJson(const ByteBuffer& buffer);
    // int read_header(ByteBuffer& buffer);
    // uint32_t time;
    // float game_time;
};

nlohmann::json BufferToJson(const ByteBuffer& buffer){
    std::string json_string(reinterpret_cast<char*>(buffer.data()), buffer.size());
    return nlohmann::json::parse(json_string);
}

// todo need space for the header: length, wall time, game time
ByteBuffer DataFrame::JsonToBuffer(const nlohmann::json& frame){
    std::string raw = frame.dump();
    ByteBuffer buffer;
    buffer.write((uint8_t*)raw.c_str(), raw.size());
    return buffer;
}

// for now 8 bit only, todo: add float higher bit rate etc enum
class ImageFrame : DataFrame{
public:
    ImageFrame(int x_res, int y_res) : channels(channels){
        resolution.x = x_res;
        resolution.y = y_res;
        data = new uint8_t[channels * resolution.x * resolution.y];
    }
    ~ImageFrame(){
        delete data;
    }
    uint8_t* data;
    int channels;
    struct Resolution{
        int x;
        int y;
    } resolution;
    int size(){
        return resolution.x * resolution.y * channels;
    }
    virtual void parse(ByteBuffer& buffer){
        // std::copy((uint8_t)buffer.data(), (uint8_t*)buffer.data() + buffer.size(), data);
        memcpy(data, buffer.data(), buffer.size());
    }
    virtual ByteBuffer write() const override{
        // todo
        return ByteBuffer();
    }
};

class RadarTargetListFrame : DataFrame{
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

class ImuFrame : DataFrame{
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

struct Quaternion{
    float x, y, z, w;
    static Quaternion extract(const nlohmann::json& frame){
        Quaternion q;
        q.x = frame[0].get<float>();
        q.y = frame[1].get<float>();
        q.z = frame[2].get<float>();
        q.w = frame[2].get<float>();
        return q;
    }
};

struct Vec3{
    float x, y, z;
    static Vec3 extract(const nlohmann::json& frame){
        Vec3 v;
        v.x = frame[0].get<float>();
        v.y = frame[1].get<float>();
        v.z = frame[2].get<float>();
        return v;
    }
};

struct Transform{
    Vec3 position;
    Quaternion orientation;
    static Transform extract(const nlohmann::json& frame){
        Transform t;
        t.position = Vec3::extract(frame["position"]);
        t.orientation = Quaternion::extract(frame["orientation"]);
        return t;
    }
};

struct OOBB{
    std::string name;
    Quaternion orientation;
    Vec3 center;
    Vec3 scale;
    Vec3 extents;
    static OOBB extract(const nlohmann::json& frame){
        OOBB obb;
        obb.name = frame["name"].get<std::string>();
        obb.orientation = Quaternion::extract(frame["orientation"]);
        obb.center = Vec3::extract(frame["center"]);
        obb.scale = Vec3::extract(frame["scale"]);
        obb.extents = Vec3::extract(frame["extents"]);
        return obb;
    }
};

struct WheelState{
    int32_t id;
    float speed;
    Transform transform;
    static WheelState extract(const nlohmann::json& frame){
        WheelState ws;
        ws.transform = Transform::extract(frame["transform"]);
        ws.speed = frame["speed"].get<float>();
        ws.id = frame["id"].get<int32_t>();
        return ws;
    }
};

struct Odometry{
    Transform pose;
    Vec3 linear_velocity;
    Vec3 angular_velocity;
    static Odometry extract(const nlohmann::json& frame){
        Odometry odo;
        odo.pose = Transform::extract(frame["transform"]);
        odo.linear_velocity = Vec3::extract(frame["linear_velocity"]);
        odo.angular_velocity = Vec3::extract(frame["angular_velocity"]);
        return odo;
    }
};

struct ObjectState{
    std::string name;
    Odometry odometry;
    std::vector<std::string> tags;
    std::vector<OOBB> oobbs;
    static ObjectState extract(const nlohmann::json& frame){
        ObjectState os;
        os.name = frame["name"].get<std::string>();
        os.odometry = Odometry::extract(frame["odometry"]);
        for(auto& tag : frame["tags"]){
            os.tags.emplace_back(tag.get<std::string>());
        }
        for(auto& obb : frame["oriented_bounding_box"]){
            os.oobbs.emplace_back(OOBB::extract(obb));
        }
        return os;
    }
};

struct VehicleState{
    ObjectState state;
    std::vector<WheelState> wheels;
    static VehicleState extract(const nlohmann::json& frame){
        VehicleState vs;
        vs.state = ObjectState::extract(frame["state"]);
        for(auto& wheel : frame["wheels"]){
            vs.wheels.emplace_back(WheelState::extract(wheel));
        }
        return vs;
    }
};

class StateFrame : DataFrame{
    virtual void parse(ByteBuffer& buffer) override;
    virtual ByteBuffer write() const override {
        // todo
        return ByteBuffer();
    }
    std::vector<VehicleState> vehicles;
    float game_time;
    int time;
};