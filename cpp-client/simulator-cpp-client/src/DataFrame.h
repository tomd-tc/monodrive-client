#include "Buffer.h"
#include "JsonHelpers.h"
#include <algorithm>
#include "Configuration.h"

class DataFrame{
public:
    virtual void parse(const ByteBuffer& buffer) = 0;
    virtual ByteBuffer write() = 0;
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
    virtual void parse(const ByteBuffer& buffer){
        std::copy(buffer.data(), buffer.data() + buffer.size(), data);
    }
};

class RadarTargetListFrame : DataFrame{
public:
    virtual void parse(const ByteBuffer& buffer) override;
    virtual ByteBuffer write() override;
    struct Target{
        std::vector<std::string> target_ids;
        float range{-1}; // m
        float aoa{0}; // degrees
        float velocity{0};
        float rcs{0}; 
    };
public:
    std::vector<Target> targets;
    std::vector<Target> gt_targets;
protected:
    void parse_target_list(const nlohmann::json& target_list);
    void parse_gt_target_list(const nlohmann::json& target_list);
    nlohmann::json write_target_list();
    nlohmann::json write_gt_target_list();
};