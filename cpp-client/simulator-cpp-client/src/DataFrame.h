#include "Buffer.h"
#include "JsonHelpers.h"
#include <algorithm>
#include "Configuration.h"

class DataFrame{
public:
    virtual void parse(const ByteBuffer& buffer) = 0;
    virtual ByteBuffer write() = 0;
    static ByteBuffer JsonToBuffer(const nlohmann::json& frame);
};

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
        // std::copy(buffer->data(), buffer()+, );
    }
};

class AnnotationFrame : DataFrame{
public:
    nlohmann::json data;
};

class RadarTargetListFrame : DataFrame{
    virtual void parse(const ByteBuffer& buffer) override;
    virtual ByteBuffer write() override;
    struct Target{
        std::vector<std::string> target_ids;
        float range{-1}; // m
        float aoa{0}; // degrees
        float velocity{0};
        float rcs{0}; 
    };
    std::vector<Target> targets;
    std::vector<Target> gt_targets;
    void parse_target_list(const nlohmann::json& target_list);
    void parse_gt_target_list(const nlohmann::json& target_list);
    nlohmann::json write_target_list();
    nlohmann::json write_gt_target_list();
};

void RadarTargetListFrame::parse_target_list(const nlohmann::json& target_list){
    targets.clear();
    int size = target_list["range"].size();
    if(size == 0)
        return;
    auto ranges = target_list["range"].get<std::vector<float>>();
    auto aoas = target_list["aoa"].get<std::vector<float>>();
    auto rcs = target_list["rcs"].get<std::vector<float>>();
    auto velocites = target_list["velocites"].get<std::vector<float>>();
    auto target_ids = target_list["target_ids"];
    for(int i = 0; i < size; ++i){
        Target target;
        for(auto& target_id : target_ids[i])
            target.target_ids.push_back(target_id["target_id"].get<std::string>());
        target.range = ranges[i];
        target.aoa = aoas[i];
        target.velocity = velocites[i];
        target.rcs = rcs[i];
        targets.emplace_back(target);
    }
}

void RadarTargetListFrame::parse_gt_target_list(const nlohmann::json& target_list){
    gt_targets.clear();
    int size = target_list["range"].size();
    if(size == 0)
        return;
    auto ranges = target_list["range"].get<std::vector<float>>();
    auto aoas = target_list["aoa"].get<std::vector<float>>();
    auto velocites = target_list["velocites"].get<std::vector<float>>();
    auto target_ids = target_list["target_ids"].get<std::vector<std::string>>();
    for(int i = 0; i < size; ++i){
        Target target;
        target.target_ids.push_back(target_ids[i]);
        target.range = ranges[i];
        target.aoa = aoas[i];
        target.velocity = velocites[i];
        gt_targets.emplace_back(target);
    }
}

void RadarTargetListFrame::parse(const ByteBuffer& buffer){
    std::string json_string(reinterpret_cast<char*>(buffer.data()), buffer.size());
    // todo: make both target list and gt list same type to simplify
    // target list
    nlohmann::json frame = json::parse(json_string);
    parse_target_list(frame["target_list"]);
    // gt target list
    parse_target_list(frame["gt_target_list"]);
}

nlohmann::json RadarTargetListFrame::write_target_list(){
    nlohmann::json target_list = nlohmann::json::array();
    for(int i = 0; i < targets.size(); ++i){
        auto& target = targets[i];
        target_list["range"].push_back(target.range);
        target_list["aoa"].push_back(target.aoa);
        target_list["velocity"].push_back(target.velocity);
        target_list["rcs"].push_back(target.rcs);
        // labview compatability with the unecessary key since variable length
        nlohmann::json target_ids = nlohmann::json::array();
        for(auto& id : target.target_ids){
            nlohmann::json target_id;
            target_id["target_id"] = id;
            target_ids.push_back(target_id);
        }
        target_list["target_ids"].push_back(target_ids);
    }
    return target_list;
}

nlohmann::json RadarTargetListFrame::write_gt_target_list(){
    nlohmann::json target_list = nlohmann::json::array();
    for(int i = 0; i < gt_targets.size(); ++i){
        auto& target = gt_targets[i];
        target_list["range"].push_back(target.range);
        target_list["aoa"].push_back(target.aoa);
        target_list["velocity"].push_back(target.velocity);
        target_list["rcs"].push_back(target.rcs);
        // labview compatability with the unecessary key since variable length
        target_list["target_ids"].push_back(target.target_ids);
    }
    return target_list;
}
ByteBuffer RadarTargetListFrame::write(){
    nlohmann::json frame;
    frame["target_list"] = write_target_list();
    frame["gt_target_list"] = write_gt_target_list();
    return JsonToBuffer(frame);
}

