#include "DataFrame.h"

void RadarTargetListFrame::parse(ByteBuffer& buffer){
    auto frame = buffer.BufferToJson()["message"];
    std::cout << frame << std::endl;
    // todo: make both target list and gt list same type to simplify
    // target list
    parse_target_list(frame["target_list"]);
    // gt target list
    parse_gt_target_list(frame["gt_target_list"]);
}

void RadarTargetListFrame::parse_target_list(const nlohmann::json& target_list){
    targets.clear();
    size_t size = target_list["ranges"].size();
    if(size == 0)
        return;
    auto ranges = target_list["ranges"].get<std::vector<float>>();
    auto aoas = target_list["aoas"].get<std::vector<float>>();
    auto rcs = target_list["rcs"].get<std::vector<float>>();
    auto velocites = target_list["velocities"].get<std::vector<float>>();
    auto target_ids = target_list["target_ids"];
    for(size_t i = 0; i < size; ++i){
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
    size_t size = target_list["ranges"].size();
    if(size == 0)
        return;
    auto ranges = target_list["ranges"].get<std::vector<float>>();
    auto aoas = target_list["aoas"].get<std::vector<float>>();
    auto velocites = target_list["velocities"].get<std::vector<float>>();
    auto target_ids = target_list["target_ids"].get<std::vector<std::string>>();
    for(size_t i = 0; i < size; ++i){
        Target target;
        target.target_ids.push_back(target_ids[i]);
        target.range = ranges[i];
        
        target.aoa = aoas[i];
        target.velocity = velocites[i];
        gt_targets.emplace_back(target);
    }
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
    return ByteBuffer::JsonToBuffer(frame);
}

ByteBuffer ImuFrame::write() const{
    // todo
    throw std::runtime_error("no implemented");
    return ByteBuffer();
}

void StateFrame::parse(ByteBuffer& buffer){
    auto j = buffer.BufferToJson()["message"];
	json_get(j, "game_time", game_time);
	json_get(j, "time", time);
	json_get(j, "sample_count", sample_count);
    auto states = j["frame"];
    auto vehicleStates = states["vehicles"];

    vehicles.clear();
	json_get(vehicleStates, vehicles);
}

ByteBuffer StateFrame::write() const {
	nlohmann::json j = {
		{"frame",
			{"time", time},
			{"game_time", game_time},
			{"sample_count", sample_count}
		}
	};
	nlohmann::json vj = nlohmann::json::array();
	for (auto& vehicle : vehicles) {
		vj.push_back(vehicle);
	}
	j["vehicles"] = vj;
	return ByteBuffer::JsonToBuffer(j);
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
    time_of_week = buffer.readInt();
}

void GPSFrame::parse(ByteBuffer& buffer){
    uint8_t preamble = buffer.readByte();
    uint16_t MSG_POS_LLH = buffer.readShort();
    id_hash = buffer.readShort();
    uint8_t payload_size = buffer.readByte();
    lattitude = buffer.readDouble();
    longitude = buffer.readDouble();
    elevation = buffer.readDouble();
    world_x = buffer.readFloat();
    world_y = buffer.readFloat();
    forward_x = buffer.readFloat();
    forward_y = buffer.readFloat();
    forward_z = buffer.readFloat();
    yaw = buffer.readFloat();
    speed = buffer.readFloat();
    horizontal_accuracy = buffer.readShort();
    vertical_accuracy = buffer.readShort();
    num_sats_signal = buffer.readByte();
    fixed_mode_status = buffer.readByte();
    crc = buffer.readShort();
}

void ImageFrame::parse(ByteBuffer& buffer){
    memcpy(pixels, buffer.data(), buffer.size());
}

ByteBuffer ImageFrame::write() const {
    ByteBuffer buffer(size(), 12); 
    buffer.write(pixels, size());
    return buffer;
}

void CameraAnnotationFrame::parse(ByteBuffer& buffer) {
    annotations.clear();
	auto frames = buffer.BufferToJson();
	for (auto& frame : frames) {
		AnnotationFrame2D annotationFrame;
		if (json_get(frame, annotationFrame)) {
			annotations.insert(std::make_pair(annotationFrame.name, annotationFrame));
		}
	}
}

ByteBuffer CameraAnnotationFrame::write() const {
	nlohmann::json j = nlohmann::json::array();
	for (auto& annotation : annotations) {
		j.push_back(annotation.second);
	}
	return ByteBuffer::JsonToBuffer(j);
}

ByteBuffer CameraFrame::write() const{
    throw std::runtime_error("Not implemented");
    return ByteBuffer();
}

void CameraFrame::parse(ByteBuffer& buffer){
    if(!bHasAnnotation or currentFrameIndex % 2 == 0){
        imageFrame->parse(buffer);
    }
    else{
        annotationFrame->parse(buffer);
    }
    currentFrameIndex++;
}