// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once

#include <string>    
#include <iostream>
#include <exception>
#include "JsonHelpers.h"
#include "DataFrame.h"
#include "Util.h"
#include "config_types.h"

class SensorBaseConfig
{
    public:
        SensorBaseConfig(){};
        virtual ~SensorBaseConfig(){};
        std::string server_ip = "127.0.0.1";
        int server_port = 8999;
        std::string type = "None";
        std::string description;
        int listen_port = 0;
        bool wait_for_fresh_frame = true;
        bool enable_streaming = true;
        Location location;
        Rotation rotation;
        struct ROS{
            bool publish_to_ros{false};
            bool advertise{true};
            std::string topic{""};
            std::string message_type{""};
            bool send_tf{false};
            int queue_size{1};
        }ros;
        friend void to_json(nlohmann::json& j,const SensorBaseConfig& config);
        friend void from_json(const nlohmann::json& j, SensorBaseConfig& config);
        virtual nlohmann::json dump(){
            return *this;
        }
        virtual DataFrame* DataFrameFactory(){
            throw std::runtime_error("SensorBaseConfig does not name a DataFrame type. Use a derived Sensor type.");
        };
};

class StateConfig : public SensorBaseConfig{
public: 
    StateConfig(){
        type = "State";
    }
    std::vector<std::string> desired_tags{};
    std::vector<std::string> undesired_tags{};
    bool debug_drawing{false};
    bool include_obb{false};
    virtual DataFrame* DataFrameFactory() override{
        return new StateFrame;
    }
    virtual nlohmann::json dump(){
        return *this;
    }
};

class LidarConfig : public SensorBaseConfig
{
public:
    LidarConfig() 
    {
        type = "Lidar";
    }
    float max_distance{8000.f};
    float horizontal_resolution{0.4f};
    float rpms{300.0f};
    int n_lasers{16};
    float reset_angle{0.0f};
    virtual DataFrame* DataFrameFactory() override{
        int rotations_per_scan = (int)(360.f / horizontal_resolution);
        int numPackets = (int)std::ceil((float)rotations_per_scan / 
        (12.f * (n_lasers == 16 ? 2 : 1)));
        return new LidarFrame(numPackets);
    }
    virtual nlohmann::json dump(){
        return *this;
    }
};

class SemanticLidarConfig : public LidarConfig {
public:
    SemanticLidarConfig()
    {
        type = "SemanticLidar";
    }
};

// todo: make sure defaults are correct
class RadarConfig : public SensorBaseConfig
{
public:
    RadarConfig()
    {
        type = "Radar";
    }
    int gpu_number{0};
    bool paint_targets{false};
    float target_paint_lifetime{0.5f};
    float nearest_target_label_radius{50.0f};
    bool send_radar_cube{false};
    double fs{50000000.0};
    double fc{77000000000.0};
    int num_sweeps{32};
    int num_samples_per_sweep{345};
    double sweep_time{0.0000069};
    int bandwidth{250000000};
    int max_radar_returns{500};
    int elements{8};
    std::string fmcw{""};
    struct Transmitter
    {
        float peak_power{5.0f};
        double aperture{0.000859};
        float gain{13.5f};
    }transmitter;
    struct Receiver
    {
        float gain{10.0f};
        double aperture{0.000798};
        float nf{10.0f};
        float noise_temp{290.0};
        double nb{74000000.0};
    }receiver;
    struct SBR
    {
        float long_range_scan_distance{60.0f};
        float short_range_scan_distance{30.0f};
        float long_range_fov{30.0f};
        float short_range_fov{60.0f};
        float elevation_fov{10.0f};
        float ray_division_y{10.0f};
        float ray_division_z{10.0f};
        float ray_division_noise_y{200.0f};
        float ray_division_noise_z{200.0f};
        bool debug_frustum{false};
        bool debug_scan{false};
        bool debug_rescan{false};
    }sbr;
    virtual DataFrame* DataFrameFactory() override{
        return new RadarFrame(send_radar_cube, num_sweeps, num_samples_per_sweep, elements);
    }
    virtual nlohmann::json dump(){
        return *this;
    }
};

class UltrasonicConfig : public SensorBaseConfig
{
public:
    UltrasonicConfig()
    {
        type = "Ultrasonic";
    }
    double fc{40000.0};
    double pwm_factor{2.5};
    float period{60.0f / 1000.0f};
    int max_ultrasonic_returns{93};
    bool send_processed_data{true};
    struct SBR
    {
        float scan_distance{4.0f};
        float azimuth_fov{30.0f};
        float elevation_fov{0.5f};
        float ray_division_y{5.0f};
        float ray_division_z{5.0f};
        bool debug_frustum{false};
        bool debug_scan{false};
        bool debug_rescan{false};
    }sbr;   
    virtual DataFrame* DataFrameFactory() override{
        return new UltrasonicFrame(send_processed_data, 1);
    }
    virtual nlohmann::json dump(){
        return *this;
    }
};

class CameraConfig : public SensorBaseConfig
{
public:
    CameraConfig()
    {
        type = "Camera";
    }
    Resolution resolution;
    float max_distance{50000.0f};
    float dynamic_range{50.0f};
    float fov{60.0f};
    float focal_length{9.0f};
    float fstop{1.4f};
    float min_shutter{.0005f};
    float max_shutter{.0014f};
    float sensor_size{9.07f};
    std::string channels{"bgra"};
    int channel_depth{1};
    struct Annotation{
        Annotation(){}
        bool include_annotation{false};
        float far_plane{10000.f};
        std::vector<std::string> desired_tags{};
        bool include_tags{false};
        bool include_oob{false};
        bool cull_partial_frame{false};
        bool debug_draw{false};
    } annotation;
    struct ColorFilterArray {
        bool use_cfa{false};
        std::string cfa{"rccc"};
    } color_filter_array;
    Viewport viewport;
    virtual DataFrame* DataFrameFactory() override{
        int nChannels = 4;
        if(channels.compare("bgra") == 0 || channels.compare("rgba") == 0)
            nChannels = 4;
        else if(channels.compare("gray") == 0)
            nChannels = 1;
        else
            throw std::runtime_error("only bgra and gray are supported channel types");
        return new CameraFrame(resolution.x, resolution.y, nChannels, 
            channel_depth, annotation.include_annotation);
    }
    virtual nlohmann::json dump(){
        return *this;
    }
};

class Camera360Config : public CameraConfig
{
public:
    Camera360Config() : CameraConfig()
    {
        type = "Camera360";
    }
    int face_size{512};
    virtual DataFrame* DataFrameFactory() override{
        return new CubeCameraFrame(resolution.x, resolution.y);
    }
    virtual nlohmann::json dump() {
        return *this;
    }
};

class Poly1FisheyeCameraConfig : public Camera360Config
{
public:
    Poly1FisheyeCameraConfig() : Camera360Config()
    {
        type = "Poly1FisheyeCamera";
        fov = 180.f;
    }
    float a0, a2, a3, a4;
    virtual nlohmann::json dump(){
        return *this;
    }
};

class EquidistantFisheyeCameraConfig : public Camera360Config
{
public:
    EquidistantFisheyeCameraConfig() : Camera360Config()
    {
        type = "EquidistantFisheyeCamera";
        fov = 180.f;
        fisheye_pixel_diameter = std::min(resolution.x, resolution.y);
    }
    EquidistantFisheyeCameraConfig (Resolution res) : Camera360Config(){
        type = "EquidistantFisheyeCamera";
        fov = 180.f;
        resolution = res;
        fisheye_pixel_diameter = std::min(resolution.x, resolution.y);
    }
    int fisheye_pixel_diameter;
    float vignette_radius_start = 0.95f;
    float vignette_bias = 0.5f;
    virtual nlohmann::json dump() {
        return *this;
    }
};

class SemanticCameraConfig : public CameraConfig
{
public:
    SemanticCameraConfig() : CameraConfig()
    {
        type = "SemanticCamera";
        channels = "gray";
    }
};

class DepthCameraConfig : public CameraConfig
{
public:
    DepthCameraConfig() : CameraConfig()
    {
        type = "DepthCamera";
        channels = "gray";
        channel_depth = 4;
    }
};

class OccupancyGridConfig : public SemanticCameraConfig
{
public:
    OccupancyGridConfig() : SemanticCameraConfig()
    {
        type = "OccupancyGrid";
        channels = "gray";
        rotation = Rotation(0.0, -90.0, 0.0);
        location = Location(0.0, 0.0, 0.0);
    }
    double meters_per_pixel = 0.1;
    bool follow_yaw = false;
    bool follow_pitch = false;
    bool follow_roll = false;

    virtual DataFrame* DataFrameFactory() override{
        int nChannels = 1;
        int channelDepth = 1;
        return new CameraFrame(resolution.x, resolution.y, nChannels, channelDepth, false);
    }
    virtual nlohmann::json dump(){
        return *this;
    }
};

class GPSConfig : public SensorBaseConfig
{
public:
    GPSConfig()
    {
        type = "GPS";
    }
    virtual DataFrame* DataFrameFactory() override{
        return new GPSFrame;
    }
};

class IMUConfig : public SensorBaseConfig
{
public:
    IMUConfig()
    {
        type = "IMU";
    }
    virtual DataFrame* DataFrameFactory() override{
        return new ImuFrame;
    }
};

class WaypointConfig : public SensorBaseConfig
{
public:
    WaypointConfig()
    {
        type = "Waypoint";
    }
    virtual DataFrame* DataFrameFactory() override 
    {
        return new WaypointFrame;
    }
    virtual nlohmann::json dump()
    {
        return *this;
    }

    float distance = 1000.0;
    float frequency = 100.0;
    bool draw_debug = false;
    std::vector<std::string> debug_tags{};
};

class RPMConfig : public SensorBaseConfig
{
public:
    RPMConfig()
    {
        type = "RPM";
    }
    int wheel_number = 0;
    virtual DataFrame* DataFrameFactory() override {
        return new RPMFrame(wheel_number);
    }
    virtual nlohmann::json dump(){
        return *this;
    }
};

class CollisionConfig : public SensorBaseConfig
{
public:
    CollisionConfig()
    {
        type = "Collision";
    }
    std::vector<std::string> desired_tags{"vt"};
    std::vector<std::string> undesired_tags{"static"};
    virtual nlohmann::json dump() { return *this; }
    virtual DataFrame* DataFrameFactory() override{
        return new CollisionFrame;
    }
};


class ViewportCameraConfig : public CameraConfig
{
public:
    ViewportCameraConfig()
    {
        type = "ViewportCamera";
        enable_streaming = false;
    }
    bool enable_hud = false;

    virtual DataFrame* DataFrameFactory() override {
        return nullptr;
    }
    virtual nlohmann::json dump() {
        return *this;
    }
};
/// SensorBaseConfig
void inline to_json(nlohmann::json& j, const SensorBaseConfig::ROS& ros)
{
    j = nlohmann::json{   
        {"publish_to_ros", ros.publish_to_ros},
        {"advertise", ros.advertise},
        {"topic", ros.topic},
        {"message_type", ros.message_type},
        {"send_tf", ros.send_tf},
        {"queue_size", ros.queue_size}
    };
}
void inline from_json(const nlohmann::json& j, SensorBaseConfig::ROS& ros)
{
    json_get(j,"publish_to_ros", ros.publish_to_ros);
    json_get(j,"advertise", ros.advertise);
    json_get(j,"topic", ros.topic);
    json_get(j,"message_type", ros.message_type);
    json_get(j,"send_tf", ros.send_tf);
    json_get(j, "queue_size", ros.queue_size);
}
void inline to_json(nlohmann::json& j, const SensorBaseConfig& config)
{
    j = nlohmann::json{
        {"type", config.type},
        {"description", config.description},
        {"listen_port", config.listen_port},
        {"location", config.location},
        {"rotation", config.rotation},
        {"wait_for_fresh_frame", config.wait_for_fresh_frame},
        {"enable_streaming", config.enable_streaming},
        {"ros", config.ros},
    };
};
void inline from_json(const nlohmann::json& j, SensorBaseConfig& config)
{
    json_get(j, "type", config.type);
    json_get(j, "description", config.description);
    json_get(j, "listen_port", config.listen_port);
    json_get(j, "location", config.location);
    json_get(j, "rotation", config.rotation);
    json_get(j, "wait_for_fresh_frame", config.wait_for_fresh_frame);
    json_get(j, "enable_streaming", config.enable_streaming);
    json_get(j, "ros", config.ros);
}
/// End SensorBaseConfig JSON Parsing

/// StateConfig JSON Parsing
void inline to_json(nlohmann::json& j, const StateConfig& config)
{
    j = static_cast<SensorBaseConfig>(config);
    j["desired_tags"] = config.desired_tags;
    j["undesired_tags"] = config.undesired_tags;
    j["debug_drawing"] = config.debug_drawing;
    j["include_obb"] = config.include_obb;
}

void inline from_json(const nlohmann::json& j, StateConfig& config)
{
    SensorBaseConfig* base = static_cast<SensorBaseConfig*>(&config);
    from_json(j, *base);

    json_get(j, "include_obb", config.include_obb);
    json_get(j, "debug_drawing", config.debug_drawing);
    json_get(j, "desired_tags", config.desired_tags);
    json_get(j, "undesired_tags", config.undesired_tags);
}
/// End StateConfig JSON Parsing

/// Camera Config JSON Parsing
void inline to_json(nlohmann::json& j, const CameraConfig::Annotation& annotation){
    j = nlohmann::json{
        {"include_annotation", annotation.include_annotation},
        {"far_plane", annotation.far_plane},
        {"cull_partial_frame", annotation.cull_partial_frame},
        {"desired_tags", annotation.desired_tags},
        {"include_obb", annotation.include_oob},
        {"include_tags", annotation.include_tags},
        {"debug_draw", annotation.debug_draw}
    };
}

void inline from_json(const nlohmann::json& j, CameraConfig::Annotation& annotation)
{
    json_get(j, "include_annotation", annotation.include_annotation);
    json_get(j, "far_plane", annotation.far_plane);
    json_get(j, "cull_partial_frame", annotation.cull_partial_frame);
    json_get(j, "desired_tags", annotation.desired_tags);
    json_get(j, "include_obb", annotation.include_oob);
    json_get(j, "include_tags", annotation.include_tags);
    json_get(j, "debug_draw", annotation.debug_draw);
}

void inline to_json(nlohmann::json& j, const CameraConfig::ColorFilterArray& cfa) {
    j = nlohmann::json{
        {"cfa", cfa.cfa},
        {"use_cfa", cfa.use_cfa}
    };
}
void inline from_json(const nlohmann::json& j, CameraConfig::ColorFilterArray& cfa) {
    json_get(j, "cfa", cfa.cfa);
    json_get(j, "use_cfa", cfa.use_cfa);
}

/// Camera Config JSON Parsing

void inline to_json(nlohmann::json& j, const CameraConfig& config)
{
    j = static_cast<SensorBaseConfig>(config);
    j["stream_dimensions"] = config.resolution;
    j["max_distance"] = config.max_distance;
    j["dynamic_range"] = config.dynamic_range;
    j["fov"]  = config.fov;
    j["focal_length"] = config.focal_length;
    j["fstop"] = config.fstop;
    j["min_shutter"] = config.min_shutter;
    j["max_shutter"] = config.max_shutter;
    j["sensor_size"] = config.sensor_size;
    j["channels"] = config.channels;
    j["channel_depth"] = config.channel_depth;
    j["annotation"] = config.annotation;
    j["color_filter_array"] = config.color_filter_array;
    j["viewport"] = config.viewport;
}

void inline from_json(const nlohmann::json& j, CameraConfig& config)
{
    SensorBaseConfig* base = static_cast<SensorBaseConfig*>(&config);
    from_json(j, *base);

    json_get(j, "stream_dimensions", config.resolution),
    json_get(j, "max_distance", config.max_distance);
    json_get(j, "dynamic_range", config.dynamic_range);
    json_get(j, "fov", config.fov);    
    json_get(j, "focal_length", config.focal_length);
    json_get(j, "fstop", config.fstop);
    json_get(j, "min_shutter", config.min_shutter); 
    json_get(j, "max_shutter", config.max_shutter); 
    json_get(j, "sensor_size", config.sensor_size); 
    json_get(j, "channels", config.channels);         
    json_get(j, "channel_depth", config.channel_depth);         
    json_get(j, "annotation", config.annotation);
    json_get(j, "color_filter_array", config.color_filter_array);
    json_get(j, "viewport", config.viewport);
}

void inline to_json(nlohmann::json& j, const Camera360Config& config)
{
    j = static_cast<CameraConfig>(config);
    j["face_size"] = config.face_size;
}

void inline from_json(const nlohmann::json& j, Camera360Config& config)
{
    CameraConfig* base = static_cast<CameraConfig*>(&config);
    from_json(j, *base);

    json_get(j, "face_size", config.face_size);
}

void inline to_json(nlohmann::json& j, const Poly1FisheyeCameraConfig& config)
{
    j = static_cast<Camera360Config>(config);
    j["a0"] = config.a0;
    j["a2"] = config.a2;
    j["a3"] = config.a3;
    j["a4"] = config.a4;
}

void inline from_json(const nlohmann::json& j, Poly1FisheyeCameraConfig& config)
{
    Camera360Config* base = static_cast<Camera360Config*>(&config);
    from_json(j, *base);

    json_get(j, "a0", config.a0);
    json_get(j, "a2", config.a2);
    json_get(j, "a3", config.a3);
    json_get(j, "a4", config.a4);
}


void inline to_json(nlohmann::json& j, const EquidistantFisheyeCameraConfig& config)
{
    j = static_cast<Camera360Config>(config);
    j["fisheye_pixel_diameter"] = config.fisheye_pixel_diameter;
    j["vignette_bias"] = config.vignette_bias;
    j["vignette_radius_start"] = config.vignette_radius_start;
}

void inline from_json(const nlohmann::json& j, EquidistantFisheyeCameraConfig& config)
{
    Camera360Config* base = static_cast<Camera360Config*>(&config);
    from_json(j, *base);

    json_get(j, "fisheye_pixel_diameter", config.fisheye_pixel_diameter);
    json_get(j, "vignette_bias", config.vignette_bias);
    json_get(j, "vignette_radius_start", config.vignette_radius_start);
}

void inline to_json(nlohmann::json& j, const ViewportCameraConfig& config)
{
    j = static_cast<CameraConfig>(config);
    j["use_vehicle_hud"] = config.enable_hud;
}

void inline from_json(const nlohmann::json& j, ViewportCameraConfig& config)
{
    CameraConfig* base = static_cast<CameraConfig*>(&config);
    from_json(j, *base);
    json_get(j, "use_vehicle_hud", config.enable_hud);
}

/// END Camera Config JSON Parsing

/// Radar Config JSON Parsing
void inline to_json(nlohmann::json& j, const RadarConfig::SBR& config)
{
    j = nlohmann::json{
        {"long_range_scan_distance", config.long_range_scan_distance},
        {"short_range_scan_distance", config.short_range_scan_distance},
        {"long_range_fov", config.long_range_fov},
        {"short_range_fov", config.short_range_fov},
        {"elevation_fov", config.elevation_fov},
        {"ray_division_z", config.ray_division_z},
        {"ray_division_y", config.ray_division_y},
        {"ray_division_noise_z", config.ray_division_noise_z},
        {"ray_division_noise_y", config.ray_division_noise_y},
        {"debug_frustum", config.debug_frustum},
        {"debug_scan", config.debug_scan},
        {"debug_rescan", config.debug_rescan}
      };
}
void inline from_json(const nlohmann::json& j, RadarConfig& config)
{
    SensorBaseConfig* base = static_cast<SensorBaseConfig*>(&config);
    from_json(j, *base);

    json_get(j, "paint_targets", config.paint_targets);
    json_get(j, "target_paint_lifetime", config.target_paint_lifetime);
    json_get(j, "nearest_target_label_radius", config.nearest_target_label_radius);
    json_get(j, "send_radar_cube", config.send_radar_cube);
    json_get(j, "gpu_number", config.gpu_number);
    json_get(j, "fs", config.fs);
    json_get(j, "fc", config.fc);
    json_get(j, "num_sweeps", config.num_sweeps);
    json_get(j, "num_samples_per_sweep", config.num_samples_per_sweep);
    json_get(j, "sweep_time", config.sweep_time);
    json_get(j, "max_radar_returns", config.max_radar_returns);
    json_get(j, "elements", config.elements);
    json_get(j, "transmitter", config.transmitter);
    json_get(j, "receiver", config.receiver);
    json_get(j, "sbr", config.sbr);
}

void inline from_json(const nlohmann::json& j, RadarConfig::Transmitter& config)
{
    json_get(j, "peak_power", config.peak_power);
    json_get(j, "aperture", config.aperture);
    json_get(j, "gain", config.gain);
}
void inline to_json(nlohmann::json& j, const RadarConfig::Transmitter& config)
{
     j = nlohmann::json{
        {"peak_power", config.peak_power},
        {"aperture", config.aperture},
        {"gain", config.gain}
     };
}

void inline from_json(const nlohmann::json& j, RadarConfig::Receiver& config)
{
    json_get(j, "gain", config.gain);
    json_get(j, "aperture", config.aperture);
    json_get(j, "nf", config.nf);
    json_get(j, "noise_temp", config.noise_temp);
    json_get(j, "nb", config.nb);
}
void inline to_json(nlohmann::json& j, const RadarConfig::Receiver& config)
{
    j = nlohmann::json{
        {"gain", config.gain},
        {"aperture", config.aperture},
        {"nf", config.nf},
        {"noise_temp", config.noise_temp},
        {"nb", config.nb}    
    };
}

void inline from_json(const nlohmann::json& j, RadarConfig::SBR& config)
{
    json_get(j, "long_range_scan_distance", config.long_range_scan_distance);
    json_get(j, "short_range_scan_distance",config.short_range_scan_distance);
    json_get(j, "long_range_fov",           config.long_range_fov);
    json_get(j, "short_range_fov",          config.short_range_fov);
    json_get(j, "elevation_fov",            config.elevation_fov);
    json_get(j, "ray_division_y",           config.ray_division_y);
    json_get(j, "ray_division_z",           config.ray_division_z);
    json_get(j, "debug_frustum",           config.debug_frustum);
    json_get(j, "debug_scan",           config.debug_scan);
    json_get(j, "debug_rescan",           config.debug_rescan);
}

void inline to_json(nlohmann::json& j, const RadarConfig& config)
{
    j = static_cast<SensorBaseConfig>(config);
    j["paint_targets"] = config.paint_targets;
    j["target_paint_lifetime"] = config.target_paint_lifetime;
    j["nearest_target_label_radius"] = config.nearest_target_label_radius;
    j["send_radar_cube"] = config.send_radar_cube;
    j["gpu_number"] = config.gpu_number;
    j["fs"] = config.fs;
    j["fc"] = config.fc;
    j["num_sweeps"] = config.num_sweeps;
    j["num_samples_per_sweep"] = config.num_samples_per_sweep;
    j["sweep_time"] = config.sweep_time;
    j["bandwidth"] = config.bandwidth;
    j["max_radar_returns"] = config.max_radar_returns;
    j["elements"] = config.elements;
    j["transmitter"] = config.transmitter;
    j["receiver"] = config.receiver;
    j["sbr"] = config.sbr;
}

/// END Radar Config JSON Parsing

/// Ultrasonic Config JSON Parsing
void inline to_json(nlohmann::json& j, const UltrasonicConfig::SBR& config) {
    j = nlohmann::json{
        {"scan_distance", config.scan_distance},
        {"azimuth_fov", config.azimuth_fov},
        {"elevation_fov", config.elevation_fov},
        {"ray_division_y", config.ray_division_y},
        {"ray_division_z", config.ray_division_z},
        {"debug_frustum", config.debug_frustum},
        {"debug_scan", config.debug_scan},
        {"debug_rescan", config.debug_rescan}
    };
}
void inline from_json(const nlohmann::json& j, UltrasonicConfig::SBR& config) {
    json_get(j, "scan_distance", config.scan_distance);
    json_get(j, "azimuth_fov", config.azimuth_fov);
    json_get(j, "elevation_fov", config.elevation_fov);
    json_get(j, "ray_division_y", config.ray_division_y);
    json_get(j, "ray_division_z", config.ray_division_z);
    json_get(j, "debug_frustum", config.debug_frustum);
    json_get(j, "debug_scan", config.debug_scan);
    json_get(j, "debug_rescan", config.debug_rescan);
}
void inline to_json(nlohmann::json& j, const UltrasonicConfig& config) {
    j = static_cast<SensorBaseConfig>(config);
    j["fc"] = config.fc;
    j["pwm_factor"] = config.pwm_factor;
    j["max_ultrasonic_returns"]  = config.max_ultrasonic_returns;
    j["send_processed_data"]  = config.send_processed_data;
    j["sbr"] = config.sbr;
}
void inline from_json(const nlohmann::json& j, UltrasonicConfig& config) {
    SensorBaseConfig* base = static_cast<SensorBaseConfig*>(&config);
    from_json(j, *base);

    json_get(j, "fc", config.fc);
    json_get(j, "pwm_factor", config.pwm_factor);
    json_get(j, "max_ultrasonic_returns", config.max_ultrasonic_returns);
    json_get(j, "send_processed_data", config.send_processed_data);
    json_get(j, "sbr", config.sbr);
}
/// END Ultrasonic Config JSON Parsing

/// Lidar Config JSON Parsing
void inline to_json(nlohmann::json& j, const LidarConfig& config)
{
    j = static_cast<SensorBaseConfig>(config);
    j["max_distance"] =          config.max_distance;
    j["horizontal_resolution"] = config.horizontal_resolution;
    j["rpms"] =                  config.rpms;
    j["n_lasers"] =              config.n_lasers;
    j["reset_angle"] =           config.reset_angle;
}

void inline from_json(const nlohmann::json& j, LidarConfig& config)
{
    SensorBaseConfig* base = static_cast<SensorBaseConfig*>(&config);
    from_json(j, *base);

    json_get(j, "max_distance",         config.max_distance);
    json_get(j, "horizontal_resolution",config.horizontal_resolution);
    json_get(j, "rpms",                 config.rpms);
    json_get(j, "n_lasers",             config.n_lasers),
    json_get(j, "reset_angle",          config.reset_angle);
}

/// END Lidar Config JSON Parsing

/// GPS Config JSON Parsing

void inline to_json(nlohmann::json& j, const GPSConfig& config)
{
    j = static_cast<SensorBaseConfig>(config);
}

void inline from_json(const nlohmann::json& j, GPSConfig& config)
{
    SensorBaseConfig* base = static_cast<SensorBaseConfig*>(&config);
    from_json(j, *base);
}

/// END GPS Config JSON Parsing

/// Collision Config JSON Parsing
void inline to_json(nlohmann::json& j, const CollisionConfig& config)
{
    j = static_cast<SensorBaseConfig>(config);
    for (auto iter = config.desired_tags.begin(); iter != config.desired_tags.end(); ++iter)
    {
        j["desired_tags"].push_back(*iter);
    }
    for (auto iter = config.undesired_tags.begin(); iter != config.undesired_tags.end(); ++iter)
    {
        j["undesired_tags"].push_back(*iter);
    }
}

void inline from_json(const nlohmann::json& j, CollisionConfig& config)
{
    SensorBaseConfig* base = static_cast<SensorBaseConfig*>(&config);
    from_json(j, *base);

    json_get(j, "desired_tags", config.desired_tags);
    json_get(j, "undesired_tags", config.undesired_tags);   
}
/// END Collision Config JSON Parsing

/// Occupancy Grid Sensor JSON parsing
void inline to_json(nlohmann::json& j, const OccupancyGridConfig& config)
{
    j = static_cast<SemanticCameraConfig>(config);
    j["stream_dimensions"] = config.resolution;
    j["meters_per_pixel"] = config.meters_per_pixel;
    j["follow_yaw"] = config.follow_yaw;
    j["follow_pitch"] = config.follow_pitch;
    j["follow_roll"] = config.follow_roll;
}
void inline from_json(const nlohmann::json& j, OccupancyGridConfig& config)
{
    SemanticCameraConfig* base = static_cast<SemanticCameraConfig*>(&config);
    from_json(j, *base);

    json_get(j, "stream_dimensions", config.resolution);
    json_get(j, "meters_per_pixel", config.meters_per_pixel);
    json_get(j, "follow_yaw", config.follow_yaw);
    json_get(j, "follow_pitch", config.follow_pitch);
    json_get(j, "follow_roll", config.follow_roll);
}
/// END Occupancy Grid Sensor JSON parsing

/// RPM Sensor JSON parsing
void inline to_json(nlohmann::json& j, const RPMConfig& config) 
{
    j = static_cast<SensorBaseConfig>(config);
    j["wheelNumber"] = config.wheel_number;
}

void inline from_json(const nlohmann::json& j, RPMConfig& config)
{
    SensorBaseConfig* base = static_cast<SensorBaseConfig*>(&config);
    from_json(j, *base);

    json_get(j, "wheelNumber", config.wheel_number);
}
/// END RPM Sensor JSON parsing

/// Waypoint Sensor JSON parsing
void inline to_json(nlohmann::json& j, const WaypointConfig& config) {
    j = static_cast<SensorBaseConfig>(config);
    j["distance"] = config.distance;
    j["frequency"] = config.frequency;
    j["draw_debug"] = config.draw_debug;
    j["debug_tags"] = config.debug_tags;

}
void inline from_json(const nlohmann::json& j, WaypointConfig& config) {
    json_get(j, "distance", config.distance);
    json_get(j, "frequency", config.frequency);
    json_get(j, "draw_debug", config.draw_debug);
    json_get(j, "debug_tags", config.debug_tags);
}
/// END Waypoint Sensor JSON parsing


std::unique_ptr<SensorBaseConfig> inline sensorConfigFactory(const nlohmann::json& j)
{
    std::string sensorType;
    json_get(j, "type", sensorType);

    if (sensorType == "Camera") {
        CameraConfig cfg;
        from_json(j, cfg);
        return make_unique<CameraConfig>(cfg);
    }
    else if (sensorType == "SemanticCamera") {
        SemanticCameraConfig cfg;
        from_json(j, cfg);
        return make_unique<SemanticCameraConfig>(cfg);
    }
    else if (sensorType == "Camera360") {
        Camera360Config cfg;
        from_json(j, cfg);
        return make_unique<Camera360Config>(cfg);
    }
    else if (sensorType == "Poly1FisheyeCamera") {
        Poly1FisheyeCameraConfig cfg;
        from_json(j, cfg);
        return make_unique<Poly1FisheyeCameraConfig>(cfg);
    }
    else if (sensorType == "EquidistantFisheyeCamera") {
        EquidistantFisheyeCameraConfig cfg;
        from_json(j, cfg);
        return make_unique<EquidistantFisheyeCameraConfig>(cfg);
    }
    else if (sensorType == "Lidar") {
        LidarConfig cfg;
        from_json(j, cfg);
        return make_unique<LidarConfig>(cfg);
    }
    else if (sensorType == "SemanticLidar") {
        SemanticLidarConfig cfg;
        from_json(j, cfg);
        return make_unique<SemanticLidarConfig>(cfg);
    }
    else if (sensorType == "Radar") {
        RadarConfig cfg;
        from_json(j, cfg);
        return make_unique<RadarConfig>(cfg);
    }
    else if (sensorType == "Ultrasonic") {
        UltrasonicConfig cfg;
        from_json(j, cfg);
        return make_unique<UltrasonicConfig>(cfg);
    }
    else if (sensorType == "RadarGroundTruth") {
        return nullptr;
    }
    else if (sensorType == "GPS") {
        GPSConfig cfg;
        from_json(j, cfg);
        return make_unique<GPSConfig>(cfg);
    }
    else if (sensorType == "IMU") {
        IMUConfig cfg;
        from_json(j, cfg);
        return make_unique<IMUConfig>(cfg);
    }
    else if (sensorType == "State") {
        StateConfig cfg;
        from_json(j, cfg);
        return make_unique<StateConfig>(cfg);
    }
    else if (sensorType == "Waypoint") {
        WaypointConfig cfg;
        from_json(j, cfg);
        return make_unique<WaypointConfig>(cfg);
    }
    else if (sensorType == "RPM") {
        RPMConfig cfg;
        from_json(j, cfg);
        return make_unique<RPMConfig>(cfg);
    }
    else if (sensorType == "Collision") {
        CollisionConfig cfg;
        from_json(j, cfg);
        return make_unique<CollisionConfig>(cfg);
    }
    else if (sensorType == "DepthCamera") {
        DepthCameraConfig cfg;
        from_json(j, cfg);
        return make_unique<DepthCameraConfig>(cfg);
    }
    else if (sensorType == "OccupancyGrid") {
        OccupancyGridConfig cfg;
        from_json(j, cfg);
        return make_unique<OccupancyGridConfig>(cfg);
    }
    else if (sensorType == "ViewportCamera") {
        ViewportCameraConfig cfg;
        from_json(j, cfg);
        return make_unique<ViewportCameraConfig>(cfg);
    }
    return nullptr;
}
