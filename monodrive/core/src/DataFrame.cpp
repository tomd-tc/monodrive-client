// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#include "DataFrame.h"
#include "Stopwatch.h"

#define IMU_DATA_PACKET_SIZE 35
#define GPS_DATA_PACKET_SIZE 66
#define LIDAR_PACKET_SIZE 1206

ByteBuffer UltrasonicTargetListFrame::write() const{
    nlohmann::json frame = {
        {"targets", targets}
    };
    ByteBuffer buffer = ByteBuffer::JsonToBuffer(frame);
    write_mono_header(buffer);
    return buffer;
}

void UltrasonicTargetListFrame::parse(ByteBuffer& buffer){
    auto frame = buffer.BufferToJson();
    json_get(frame, "targets", targets);
}

ByteBuffer UltrasonicRawFrame::write() const{
    ByteBuffer buffer(ultrasonic_raw.size()*sizeof(float), DATA_FRAME_HEADER_SIZE);
    auto buffer_data = reinterpret_cast<float*>(buffer.data());
    std::copy(ultrasonic_raw.data(), ultrasonic_raw.data()+ultrasonic_raw.size(), buffer_data);
    write_mono_header(buffer);
    return buffer;
}

void UltrasonicFrame::parse(ByteBuffer& buffer){
    if(bSendUltrasonicRaw and currentFrameIndex % 2 == 1){
        ultrasonicRawFrame->parse_header(buffer);
        ultrasonicRawFrame->parse(buffer);
    }
    else{
        ultrasonicTargetListFrame->parse_header(buffer);
        ultrasonicTargetListFrame->parse(buffer);
    }
    currentFrameIndex++;
}

void UltrasonicRawFrame::parse(ByteBuffer& buffer){
    auto data = reinterpret_cast<float*>(buffer.data());
    std::copy(data, data+ultrasonic_raw.size(), ultrasonic_raw.data());
}

ByteBuffer UltrasonicFrame::write() const{
    throw std::runtime_error("Not implemented. Use its member frames.");
    return ByteBuffer();
}

void RadarTargetListFrame::parse(ByteBuffer& buffer){
    auto frame = buffer.BufferToJson();
    json_get(frame, "target_list", targets);
    json_get(frame, "gt_targets", gt_targets);
}

ByteBuffer RadarTargetListFrame::write() const{
    nlohmann::json frame = {
        {"target_list", targets},
        {"gt_targets", gt_targets}
    };
    ByteBuffer buffer = ByteBuffer::JsonToBuffer(frame);
    write_mono_header(buffer);
    return buffer;
}

ByteBuffer RadarCubeFrame::write() const{
    ByteBuffer buffer(size()*sizeof(std::complex<float>), DATA_FRAME_HEADER_SIZE);
    auto buffer_data = reinterpret_cast<std::complex<float>*>(buffer.data());
    std::copy(radar_cube.data(), radar_cube.data()+size(), buffer_data);
    write_mono_header(buffer);
    return buffer;
}

void RadarCubeFrame::parse(ByteBuffer& buffer){
    auto data = reinterpret_cast<std::complex<float>*>(buffer.data());
    std::copy(data, data+size(), radar_cube.data());
}

void RadarFrame::parse(ByteBuffer& buffer){
    if(bSendRadarCube and currentFrameIndex % 2 == 1){
        radarCubeFrame->parse_header(buffer);
        radarCubeFrame->parse(buffer);
    }
    else{
        radarTargetListFrame->parse_header(buffer);
        radarTargetListFrame->parse(buffer);
    }
    currentFrameIndex++;
}

ByteBuffer RadarFrame::write() const{
    throw std::runtime_error("Not implemented. Use its member frames.");
    return ByteBuffer();
};

void StateFrame::parse(ByteBuffer& buffer){
    auto j = buffer.BufferToJson();
	json_get(j, "game_time", game_time);
	json_get(j, "time", time);
	json_get(j, "sample_count", sample_count);
    const auto& states = j["frame"];

    vehicles.clear();
    objects.clear();
	json_get(states, "vehicles", vehicles);
	json_get(states, "objects", objects);
}

ByteBuffer StateFrame::write() const {
	nlohmann::json j = {
        {"time", time},
        {"game_time", game_time},
        {"sample_count", sample_count},
		{"frame", {
                {"vehicles", vehicles},
                {"objects", objects}
            }
		}
	};
    ByteBuffer buffer = ByteBuffer::JsonToBuffer(j);
    write_mono_header(buffer);
	return buffer;
}

void CollisionFrame::parse(ByteBuffer& buffer){
    auto j = buffer.BufferToJson();
    json_get(j, "game_time", game_time);
	json_get(j, "time", time);
	json_get(j, "sample_count", sample_count);
    from_json(j, ego_target);
	json_get(j, "targets", collision_targets);
}

ByteBuffer CollisionFrame::write() const {
	nlohmann::json j;
    to_json(j, ego_target);
    j["time"] = time;
    j["game_time"] = game_time;
    j["sample_count"] = sample_count;
    j["targets"] = collision_targets;
    ByteBuffer buffer = ByteBuffer::JsonToBuffer(j);
    write_mono_header(buffer);
	return buffer;
}

ByteBuffer ImuFrame::write() const{
    ByteBuffer buffer(IMU_DATA_PACKET_SIZE, DATA_FRAME_HEADER_SIZE);
    buffer.write(0xc2);
    buffer.writeFloat(acceleration.x);
    buffer.writeFloat(acceleration.y);
    buffer.writeFloat(acceleration.z);
    buffer.writeFloat(angular_velocity.x);
    buffer.writeFloat(angular_velocity.y);
    buffer.writeFloat(angular_velocity.z);
    buffer.writeInt(timer);
    buffer.writeShort(checksum);
    buffer.writeInt(time_of_week);
    write_mono_header(buffer);
    return buffer;
}
void ImuFrame::parse(ByteBuffer& buffer){
    // packet_size not needed, old artifact of some specific hardware
    uint8_t packet_size = buffer.readByte();
    acceleration.x = buffer.readFloat();
    acceleration.y = buffer.readFloat();
    acceleration.z = buffer.readFloat();
    angular_velocity.x = buffer.readFloat();
    angular_velocity.y = buffer.readFloat();
    angular_velocity.z = buffer.readFloat();
    // more hardware artifacts
    timer = buffer.readInt();
    checksum = buffer.readShort();
    time_of_week = buffer.readInt();
}

ByteBuffer WaypointFrame::write() const{
    nlohmann::json j = {
        {"time", time},
        {"game_time", game_time},
        {"sample_count", sample_count},
    };
    j["waypoints"] = nlohmann::json::array();
    for(auto& awp : actor_waypoints) {
        nlohmann::json awp_json;
        to_json(awp_json, awp);
        j["waypoints"].push_back(awp_json);
    }

    ByteBuffer buffer = ByteBuffer::JsonToBuffer(j);
    write_mono_header(buffer);
    return buffer;
}
void WaypointFrame::parse(ByteBuffer& buffer) {
    actor_waypoints.clear();
    auto data = buffer.BufferToJson();
    json_get(data, "time", time);
    json_get(data, "game_time", game_time);
    json_get(data, "sample_count", sample_count);
    for(auto& actor_wp : data["waypoints"]) {
        ActorWaypoints awp;
        from_json(actor_wp, awp);
        actor_waypoints.emplace_back(awp);
    }
}

void GPSFrame::parse(ByteBuffer& buffer){
    preamble = buffer.readByte();
    MSG_POS_LLH = buffer.readShort();
    id_hash = buffer.readShort();
    payload_size = buffer.readByte();
    lattitude = buffer.readDouble();
    longitude = buffer.readDouble();
    elevation = buffer.readDouble();
    world_x = buffer.readFloat();
    world_y = buffer.readFloat();
    forward.x = buffer.readFloat();
    forward.y = buffer.readFloat();
    forward.z = buffer.readFloat();
    yaw = buffer.readFloat();
    speed = buffer.readFloat();
    horizontal_accuracy = buffer.readShort();
    vertical_accuracy = buffer.readShort();
    num_sats_signal = buffer.readByte();
    fixed_mode_status = buffer.readByte();
    crc = buffer.readShort();
}

ByteBuffer GPSFrame::write() const{
    ByteBuffer buffer(GPS_DATA_PACKET_SIZE, DATA_FRAME_HEADER_SIZE);
    buffer.write(preamble);
    buffer.writeShort(MSG_POS_LLH);
    buffer.writeShort(id_hash);
    buffer.write(payload_size);
    buffer.writeDouble(lattitude);
    buffer.writeDouble(longitude);
    buffer.writeDouble(elevation);
    buffer.writeFloat(world_x);
    buffer.writeFloat(world_y);
    buffer.writeFloat(forward.x);
    buffer.writeFloat(forward.y);
    buffer.writeFloat(forward.z);
    buffer.writeFloat(yaw);
    buffer.writeFloat(speed);
    buffer.writeShort(horizontal_accuracy);
    buffer.writeShort(vertical_accuracy);
    buffer.write(num_sats_signal);
    buffer.write(fixed_mode_status);
    buffer.writeShort(crc);
    write_mono_header(buffer);
    return buffer;
}

void ImageFrame::parse(ByteBuffer& buffer){
    auto data = reinterpret_cast<uint8_t*>(buffer.data());
    std::copy(data, data+buffer.size(), pixels);
}

ByteBuffer ImageFrame::write() const {
    ByteBuffer buffer(size(), DATA_FRAME_HEADER_SIZE); 
    buffer.write(pixels, size());
    write_mono_header(buffer);
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
    ByteBuffer buffer = ByteBuffer::JsonToBuffer(j);
    write_mono_header(buffer);
	return buffer;
}

ByteBuffer CameraFrame::write() const{
    throw std::runtime_error("Not implemented. Use it's member frames.");
    return ByteBuffer();
}

void CameraFrame::parse(ByteBuffer& buffer){
    if(!bHasAnnotation or currentFrameIndex % 2 == 0){
        buffer.reset();
        imageFrame->parse_header(buffer);
        imageFrame->parse(buffer);
    }
    else{
        buffer.reset();
        annotationFrame->parse_header(buffer);
        annotationFrame->parse(buffer);
    }
    currentFrameIndex++;
}

ByteBuffer LidarFrame::write() const {
    ByteBuffer buffer(sizeof(LidarPacket)*packets.size(), DATA_FRAME_HEADER_SIZE);
    buffer.write((uint8_t*)packets.data(), sizeof(LidarPacket) * packets.size());
    write_mono_header(buffer);
    return buffer;
}

void LidarFrame::parse(ByteBuffer& buffer){
    packets[packetIndex++] = *reinterpret_cast<LidarPacket*>(buffer.data());
    if(packetIndex == packets.size())
        packetIndex = 0;
}


void RPMFrame::parse(ByteBuffer& buffer){
    wheel_number = buffer.readInt();
    speed = buffer.readFloat();
}

ByteBuffer RPMFrame::write() const {
    ByteBuffer buffer(4, DATA_FRAME_HEADER_SIZE);
    buffer.writeInt(wheel_number);
    buffer.writeFloat(speed);
    write_mono_header(buffer);
    return buffer;
}