#pragma once

#include <string>    
#include <iostream>
#include "JsonHelpers.h"

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
        struct Location
        {
            float x{0.0f};
            float y{0.0f};
            float z{0.0f};
        }location;
        struct Rotation
        {
            float yaw{0.0f};
            float pitch{0.0f};
            float roll{0.0f};
        }rotation;
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

        virtual SensorBaseConfig& get()
        {
            return *this;
        }
        virtual std::string dump_json()
        {
            nlohmann::json j = *this;
            return j.dump();
        }
};

class StateConfig : public SensorBaseConfig{
public: 
    StateConfig(){
        type = "State";
    }
    std::vector<std::string> desired_tags{"vehicle", "dynamic"};
    std::vector<std::string> undesired_tags{"static"};
    bool debug_drawing{false};
    bool include_obb{false};
    StateConfig& get()
    {
        return *this;
    }
    std::string dump_json() override{
        nlohmann::json j = *this;
        return j.dump();
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
    std::string dump_json() override{
        nlohmann::json j = *this;
        return j.dump();
    }
};

class RadarConfig : public SensorBaseConfig
{
public:
    RadarConfig()
    {
        type = "Radar";
    }
    bool paint_targets{true};
    float target_paint_lifetime{0.25f};
    int num_samples_per_sweep{1100};
    float fs{150000000.0f};
    float fc{77000000000.0f};
    int num_sweeps{32};
    float range_max{150.0f};
    float range_resolution{1.0f};
    float max_velocity{100.0f};
    float max_targets{100.0f};
    float fps{1.0};
    int elements{8};
    struct Transmitter
    {
        float peak_power{5.0f};
        float aperature{.000859f};
        float gain{13.5f};   
    }transmitter;
    struct Receiver
    {
        float gain{20.0f};
        float aperature{0.000798f};
        float nf{10.0f};
        float noise_temp{290.0};
        float nb{74000000.0};
    }receiver;
    struct SBR
    {
        float minimum_radar_distance{1.0f};
        float long_range_scan_distance{150.0f};
        float short_range_scan_distance{60.0f};
        float num_scans_azimuth{100.0f};
        float long_range_fov{20.0f};
        float short_range_fov{90.0f};
        float num_scans_elevation{20.0f};
        float elevation_fov{5.0f};
        float max_raycast_hits{5.f};
        bool debug_draw{true};
    }sbr;   
    std::string dump_json() override{
        nlohmann::json j = *this;
        return j.dump();
    }
};


class CameraConfig : public SensorBaseConfig
{
public:
    CameraConfig()
    {
        type = "Camera";
    }
    struct Resolution
    {
        Resolution(){}
        Resolution(float x, float y) : x(x), y(y) {}
        float x{512.0f};
        float y{512.0f};
    } resolution;
    float max_distance{50000.0f};
    float dynamic_range{50.0f};
    float fov{60.0f};
    float focal_length{9.0f};
    float fstop{1.4f};
    float min_shutter{.0005f};
    float max_shutter{.0014f};
    float sensor_size{9.07f};
    std::string channels{"bgra"};
    SensorBaseConfig& get() override
    {
        return *this;
    }
    std::string dump_json() override{
        nlohmann::json j = *this;
        return j.dump();
    }      
};

class GPSConfig : public SensorBaseConfig
{
public:
    GPSConfig()
    {
        type = "GPS";
    }
    //friend void to_json(nlohmann::json& j,const GPSConfig& config);
    //friend void json_get(const json& j, GPSConfig& config);
    std::string dump_json() override{
        nlohmann::json j = *this;
        return j.dump();
    }
};

class IMUConfig : public SensorBaseConfig
{
public:
    IMUConfig()
    {
        type = "IMU";
    }
    std::string dump_json() override{
        nlohmann::json j = *this;
        return j.dump();
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
    CollisionConfig& get()
    {
        return *this;
    }
    std::string dump_json() override{
        nlohmann::json j = *this;
        return j.dump();
    }
};

class ViewportCameraConfig : public CameraConfig
{
public:
    ViewportCameraConfig()
    {
        type = "ViewportCamera";
    }
};

/// SensorBaseConfig
void inline to_json(nlohmann::json& j, const SensorBaseConfig::Location& location)
{
    j = nlohmann::json{{"x", location.x},
                {"y", location.y},
                {"z", location.z}
    };
}
void inline from_json(const nlohmann::json& j, SensorBaseConfig::Location& location)
{
    json_get(j, "x", location.x);
    json_get(j, "y", location.y);
    json_get(j, "z", location.z);
}
void inline to_json(nlohmann::json& j, const SensorBaseConfig::Rotation& rotation)
{
    j = nlohmann::json{{"yaw", rotation.yaw},
                {"pitch", rotation.pitch},
                {"roll", rotation.roll}
                };
}
void inline from_json(const nlohmann::json& j, SensorBaseConfig::Rotation& rotation)
{
    json_get(j,"yaw", rotation.yaw);
    json_get(j,"pitch", rotation.pitch);
    json_get(j, "roll", rotation.roll);
}
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
    json_get(j,"publis_to_ross", ros.publish_to_ros);
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
        {"ros", config.ros}
    };
};
void inline from_json(const nlohmann::json& j, SensorBaseConfig& config)
{
    json_get(j, "type", config.type);
    json_get(j, "description", config.description);
    json_get(j, "listen_port", config.listen_port);
    json_get(j, "location", config.location);
    json_get(j, "rotation", config.rotation);
    json_get(j, "ros", config.ros);
}
/// End SensorBaseConfig JSON Parsing

/// Camera Config JSON Parsing
void inline to_json(nlohmann::json& j, const CameraConfig::Resolution& resolution)
{
    j = nlohmann::json{{"x", resolution.x},
            {"y", resolution.y}
            };
}

void inline from_json(const nlohmann::json& j, CameraConfig::Resolution& resolution)
{
    json_get(j, "x", resolution.x);
    json_get(j, "y", resolution.y);
}

void inline to_json(nlohmann::json& j, const StateConfig& config){
    j = static_cast<SensorBaseConfig>(config);
    j["desired_tags"] = config.desired_tags;
    j["undesired_tags"] = config.undesired_tags;
    j["debug_drawing"] = config.debug_drawing;
    j["include_obb"] = config.include_obb;
}

void inline from_json(const nlohmann::json& j, StateConfig& config)
{
    json_get(j, "include_obb", config.include_obb);
    json_get(j, "debug_drawing", config.debug_drawing);
    json_vector(j, "desired_tags", config.desired_tags);
    json_vector(j, "undesired_tags", config.undesired_tags);
}

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
    j["max_shutter"] = config.max_distance;
    j["sensor_size"] = config.sensor_size;
    j["channels"] =config.channels;
}

void inline from_json(const nlohmann::json& j, CameraConfig& config)
{
    json_get(j, "stream_dimensions", config.resolution),
    json_get(j, "max_distance", config.max_distance);
    json_get(j, "dynamic_range", config.dynamic_range);
    json_get(j, "fov", config.fov);    
    json_get(j, "focal_length", config.focal_length);
    json_get(j, "fstop", config.fstop);
    json_get(j, "min_shutter", config.min_shutter); 
    json_get(j, "max_shutter", config.max_distance); 
    json_get(j, "sensor_size", config.sensor_size); 
    json_get(j, "channels", config.channels);         
}

/// END Camera Config JSON Parsing

/// Radar Config JSON Parsing
void inline to_json(nlohmann::json& j, const RadarConfig& config)
{
    j = static_cast<SensorBaseConfig>(config);
    j["paint_targets"] = config.paint_targets;
    j["target_paint_lifetime"] = config.target_paint_lifetime;
    j["num_samples_per_sweep"] = config.num_samples_per_sweep;
    j["fs"] = config.fs;
    j["fc"] = config.fc;
    j["num_sweeps"] = config.num_sweeps;
    j["range_max"] = config.range_max;
    j["sweep_num_for_range_max"] = config.num_samples_per_sweep;
    j["range_resolution"]  = config.range_resolution;
    j["max_velocity"] = config.max_velocity;
    j["max_targets"] = config.max_targets;
    j["fps"] = config.fps;
    j["elements"] = config.elements;
    j["transmitter"] = config.transmitter;
    j["receiver"] = config.receiver;
    j["sbr"] = config.sbr;
}

void inline to_json(nlohmann::json& j, const RadarConfig::Transmitter& config)
{
     j = nlohmann::json{{ "peak_power", config.peak_power},
              {"aperature", config.aperature},
              {"gain", config.gain}
     };
}

void inline to_json(nlohmann::json& j, const RadarConfig::Receiver& config)
{
    j = nlohmann::json{{"gain", config.gain},
             {"aperature", config.aperature},
             {"nf", config.nf},
             {"noise_temp", config.noise_temp},
             {"nb", config.nb}    
    };
}
void inline to_json(nlohmann::json& j, const RadarConfig::SBR& config)
{
    j = nlohmann::json{{"minimum_radar_distance", config.minimum_radar_distance},
             {"long_range_scan_distance", config.long_range_scan_distance},
             {"short_range_scan_distance", config.short_range_scan_distance},
             {"num_scans_azimuth", config.num_scans_elevation},
             {"long_range_fov", config.long_range_fov},
             {"short_range_fov", config.short_range_scan_distance},
             {"num_scans_elevation", config.num_scans_elevation},
             {"elevation_fov", config.elevation_fov},
             {"max_raycast_hits", config.max_raycast_hits},
             {"debug_draw", config.debug_draw}
      };
}
void inline from_json(const nlohmann::json& j, RadarConfig& config)
{
    json_get(j, "paint_targets", config.paint_targets);
    json_get(j, "target_paint_lifetime", config.target_paint_lifetime);
    json_get(j, "num_samples_per_sweep", config.num_samples_per_sweep);
    json_get(j, "range_max", config.range_max);
    json_get(j, "range_resolution", config.range_resolution);
    json_get(j, "max_velocity", config.max_velocity);
    json_get(j, "max_targets", config.max_targets);
    json_get(j, "fps", config.fps);
    json_get(j, "elements", config.elements);
    json_get(j, "transmitter", config.transmitter);
    json_get(j, "receiver", config.receiver);
    json_get(j, "sbr", config.sbr);
}

void inline from_json(const nlohmann::json& j, RadarConfig::Transmitter config)
{
    json_get(j, "peak_power", config.peak_power);
    json_get(j, "aperature", config.aperature);
    json_get(j, "gain", config.gain);
}

void inline from_json(const nlohmann::json& j, RadarConfig::Receiver config)
{
    json_get(j, "gain", config.gain);
    json_get(j, "aperature", config.aperature);
    json_get(j, "nf", config.nf);
    json_get(j, "noise_temp", config.noise_temp);
    json_get(j, "nb", config.nb);
}

void inline from_json(const nlohmann::json& j, RadarConfig::SBR config)
{
    json_get(j, "minimum_radar_distance",   config.minimum_radar_distance);
    json_get(j, "long_range_scan_distance", config.long_range_scan_distance);
    json_get(j, "short_range_scan_distance",config.short_range_scan_distance);
    json_get(j, "num_scans_azimuth",        config.num_scans_azimuth);
    json_get(j, "long_range_fov",           config.long_range_fov);
    json_get(j, "short_range_fov",          config.short_range_fov);
    json_get(j, "num_scans_elevation",      config.num_scans_elevation);
    json_get(j, "elevation_fov",            config.elevation_fov);
    json_get(j, "max_raycast_hits",         config.max_raycast_hits);
    json_get(j, "debug_draw",               config.debug_draw);
}

/// END Radar Config JSON Parsing

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

/*void inline from_json(const json& j, GPSConfig& config)
{
    use SensorBaseClass from_json method
}*/

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
    json_get(j, "desired_tags", config.desired_tags);
    json_get(j, "undesired_tags", config.undesired_tags);   
}
/// END Collision Config JSON Parsing