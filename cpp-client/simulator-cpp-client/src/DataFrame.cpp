#include "DataFrame.h"

#define IMU_DATA_PACKET_SIZE 35

// int DataFrame::read_header(ByteBuffer& buffer){
//     // todo this probably wrong
//     int length = buffer.readInt();
//     time = buffer.readInt();
//     game_time = buffer.readInt();
//     return length;
// }

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

void RadarTargetListFrame::parse(ByteBuffer& buffer){
    auto frame = BufferToJson(buffer);
    // todo: make both target list and gt list same type to simplify
    // target list
    parse_target_list(frame["target_list"]);
    // gt target list
    parse_target_list(frame["gt_target_list"]);
}

nlohmann::json RadarTargetListFrame::write_target_list() const{
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

nlohmann::json RadarTargetListFrame::write_gt_target_list() const{
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

ByteBuffer RadarTargetListFrame::write() const{
    nlohmann::json frame;
    frame["target_list"] = write_target_list();
    frame["gt_target_list"] = write_gt_target_list();
    return JsonToBuffer(frame);
}

void ImuFrame::parse(ByteBuffer& buffer){
    // packet_size not needed, old artifact of some specific hardware
    uint8_t packet_size = buffer.readByte();
    acc_x = buffer.readFloat();
    acc_y = buffer.readFloat();
    acc_z = buffer.readFloat();
    ang_x = buffer.readFloat();
    ang_y = buffer.readFloat();
    ang_z = buffer.readFloat();
    // more hardware artifacts
    timer = buffer.readInt();
    checksum = buffer.readShort();
    time_of_week = buffer.readLong();
}