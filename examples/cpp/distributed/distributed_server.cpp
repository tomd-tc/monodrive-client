#include <functional>

#include "distributed_server.h"

using namespace distributed_server;

DistributedServer::DistributedServer(const std::string& ip_address,
                                     const int& port,
                                     const kServerType& server_type)
    : server_type(server_type) {
  // Go ahead and grab an instance of the server before configuring
  sim = &Simulator::getInstance(
      server_type == kServerType::PRIMARY ? kPrimaryConfig : kReplicaConfig, 
      ip_address, port);

  // All server ports are technically reserved as well
  kServerReservedPorts.insert(port);
}

bool DistributedServer::Configure(const std::vector<kSensorType>& sensor_types){
  // Configure the simulator
  sensors.clear();
  if (!sim->configure()) {
    std::cerr << "DistributedServer::Configure: ERROR! Unable to configure "
                 "simulator: "
              << sim->getServerIp() << ":" << sim->getServerPort() << std::endl;
    return false;
  }

  // Configure all the sensors for the simulator
  for(auto& sensor : sensor_types) {
      if(!AddSensor(sensor)) {
        std::cerr << "DistributedServer::Configure: ERROR! Unable to configure "
                     "sensor type "
                  << sensor << " on server: " << sim->getServerIp() << ":"
                  << sim->getServerPort() << std::endl;
        return false;
      }
  }
  return true;
}

bool DistributedServer::AddSensor(kSensorType sensor_type) {
  // Go through all the reserved ports and make a reservation for this sensor
  int listen_port = 8101;
  while (kServerReservedPorts.count(listen_port)) {
    listen_port += 1;
  }
  kServerReservedPorts.insert(listen_port);

  // Choose from one of our many preconfigured sensors!
  switch (sensor_type) {
    case kSensorType::BINARY_STATE: {
      StateConfig s_config;
      s_config.server_ip = sim->getServerIp();
      s_config.server_port = sim->getServerPort();
      s_config.listen_port = listen_port;
      s_config.desired_tags = {"vehicle", "dynamic"};
      s_config.include_obb = false;
      s_config.debug_drawing = true;
      s_config.send_binary_frame = true;
      sensors.emplace_back(
          std::make_shared<Sensor>(std::make_unique<StateConfig>(s_config)));
      sensors.back()->sampleCallback = std::bind(
          &DistributedServer::StateSensorCallback, this, std::placeholders::_1);
    } break;
    case kSensorType::VIEWPORT: {
      ViewportCameraConfig vp_config;
      vp_config.server_ip = sim->getServerIp();
      vp_config.server_port = sim->getServerPort();
      vp_config.location.z = 200;
      vp_config.resolution = Resolution(256, 256);
      Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();
    } break;
    case kSensorType::RADAR: {
      RadarConfig r_config;
      r_config.server_ip = sim->getServerIp();
      r_config.server_port = sim->getServerPort();
      r_config.location.x = 300.f;
      r_config.location.z = 50.f;
      r_config.paint_targets = false;
      r_config.send_radar_cube = true;
      r_config.max_radar_returns = 45;
      r_config.sbr.ray_division_y = 10.f;
      r_config.sbr.ray_division_z = 10.f;
      r_config.sbr.debug_scan = false;
      r_config.sbr.debug_rescan = false;
      r_config.sbr.debug_frustum = false;
      r_config.listen_port = listen_port;
      sensors.emplace_back(
          std::make_shared<Sensor>(std::make_unique<RadarConfig>(r_config)));
    } break;
    case kSensorType::LIDAR: {
      LidarConfig l_config;
      l_config.server_ip = sim->getServerIp();
      l_config.server_port = sim->getServerPort();
      l_config.location.x = -10.f;
      l_config.location.z = 190.f;
      l_config.horizontal_resolution = 0.1f;
      l_config.n_lasers = 16;
      l_config.listen_port = listen_port;
      sensors.emplace_back(
          std::make_shared<Sensor>(std::make_unique<LidarConfig>(l_config)));
    } break;
    case kSensorType::IMU: {
      IMUConfig imu_config;
      imu_config.server_ip = sim->getServerIp();
      imu_config.server_port = sim->getServerPort();
      imu_config.listen_port = listen_port;
      sensors.emplace_back(
          std::make_shared<Sensor>(std::make_unique<IMUConfig>(imu_config)));
    } break;
    case kSensorType::WAYPOINT: {
      WaypointConfig waypoint_config;
      waypoint_config.server_ip = sim->getServerIp();
      waypoint_config.server_port = sim->getServerPort();
      waypoint_config.distance = 1000.0;
      waypoint_config.frequency = 100.0;
      waypoint_config.listen_port = listen_port;
      sensors.emplace_back(
          std::make_shared<Sensor>(std::make_unique<WaypointConfig>(waypoint_config)));
    } break;
    default:
      std::cerr << "DistributedServer::AddSensor: ERROR! Unknown sensor type: "
                << sensor_type << " for server " << sim->getServerIp() << ":"
                << sim->getServerPort() << std::endl;
      return false;
  };

  // Actually configure the sensors and make sure it worked
  for (auto& sensor : sensors) {
    if (!sensor->configure()) {
      std::cerr << "DistributedServer::AddSensor: ERROR! Unable to "
                   "configure sensor "
                << sensor->config->type << " on port "
                << sensor->config->listen_port << " for server "
                << sim->getServerIp() << ":" << sim->getServerPort()
                << std::endl;
      return false;
    }
  }
  return true;
}

void DistributedServer::StateSensorCallback(DataFrame* frame) {
  // Grab the state data and signal that its available
  state_sensor_data_updated = true;
  if (state_data) {
    *state_data =
        static_cast<BinaryStateFrame*>(frame)->state_buffer.BufferToJson();
  }
}

bool DistributedServer::Sample(nlohmann::json* input_state_data) {
  state_data = input_state_data;
  switch (server_type) {
    case kServerType::PRIMARY:
      // For primaries, always wait for the state data to come back
      sim->sampleAll(sensors);
      while(!state_sensor_data_updated);
      state_sensor_data_updated = false;
      break;
    case kServerType::REPLICA:
      // Replicas just get a state update
      sim->stateStepSampleAll(sensors, *state_data);
      break;
    default:
      std::cerr << "DistributedServer::Sample: ERROR! Unknown server type: "
                << server_type << std::endl;
      return false;
  }
  return true;
}
