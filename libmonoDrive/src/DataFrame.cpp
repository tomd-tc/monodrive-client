#include "DataFrame.h"
#include "Stopwatch.h"

#define IMU_DATA_PACKET_SIZE 35
#define GPS_DATA_PACKET_SIZE 66
#define LIDAR_PACKET_SIZE 1206

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
    return ByteBuffer::JsonToBuffer(frame);
}

ByteBuffer RadarCubeFrame::write() const{
    ByteBuffer buffer(size()*sizeof(std::complex<float>), 12);
    auto buffer_data = reinterpret_cast<std::complex<float>*>(buffer.data());
    std::copy(radar_cube.data(), radar_cube.data()+size(), buffer_data);
    return buffer;
}

void RadarCubeFrame::parse(ByteBuffer& buffer){
    auto data = reinterpret_cast<std::complex<float>*>(buffer.data());
    std::copy(data, data+size(), radar_cube.data());
}

void RadarFrame::parse(ByteBuffer& buffer){
    if(bSendRadarCube and currentFrameIndex % 2 == 1){
        radarCubeFrame->parse(buffer);
    }
    else{
        radarTargetListFrame->parse(buffer);
    }
    currentFrameIndex++;
}

ByteBuffer RadarFrame::write() const{
    throw std::runtime_error("Not implemented");
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
	return ByteBuffer::JsonToBuffer(j);
}

ByteBuffer ImuFrame::write() const{
    ByteBuffer buffer(IMU_DATA_PACKET_SIZE, 12);
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
    ByteBuffer buffer(GPS_DATA_PACKET_SIZE,12);
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
    return buffer;
}

void ImageFrame::parse(ByteBuffer& buffer){
    auto data = reinterpret_cast<uint8_t*>(buffer.data());
    std::copy(data, data+buffer.size(), pixels);
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

ByteBuffer LidarFrame::write() const {
    ByteBuffer buffer(sizeof(LidarPacket)*packets.size());
    buffer.write((uint8_t*)packets.data(), sizeof(LidarPacket) * packets.size());
    return buffer;
}

void LidarFrame::parse(ByteBuffer& buffer){
    packets[packetIndex++].parse(buffer);
    if(packetIndex == packets.size())
        packetIndex = 0;
}

ByteBuffer LidarPacket::write() const{
    ByteBuffer buffer(sizeof(LidarPacket), 12);
    buffer.write((uint8_t*)this, sizeof(LidarPacket));
	return buffer;
}

void LidarPacket::parse(ByteBuffer& buffer){
    *this = *reinterpret_cast<LidarPacket*>(buffer.data());
}