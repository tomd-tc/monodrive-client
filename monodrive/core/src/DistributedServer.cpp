#include <functional>

#include "DistributedServer.h"

using namespace distributed_server;

DistributedServer::DistributedServer(const Configuration& config,
                                     const std::string& ip_address,
                                     const int& port) : server_config(config) {
  // Make sure to copy the desired IP and port to the configurations since we
  // may be getting a resused config file
  server_config.simulator["server_ip"] = ip_address;
  server_config.simulator["server_port"] = port;
  // Set all the sensors to this server's IP address so we don't use a value
  // from the shared config files.
  for(auto& sensor : server_config.sensorsConfig) {
    std::cout << "Type:" << sensor["type"] << std::endl;
    sensor["server_ip"] = ip_address;
    sensor["server_port"] = port;
  }

  // Go ahead and grab an instance of the server before configuring
  sim = &Simulator::getInstance(server_config, ip_address, port);

  // All server ports are technically reserved as well
  kServerReservedPorts.insert(port);
}

bool DistributedServer::Configure(){
  // Configure the simulator
  sensors.clear();

  // Read in all the sensor configurations for this server
  if (!server_config.loadSensors(sensors)) {
    std::cerr << "DistributedServer::Configure: ERROR! Unable to configure one "
                 "or more sensors on server: "
              << sim->getServerIp() << ":" << sim->getServerPort() << std::endl;
    return false;
  }

  if (!sim->configure()) {
    std::cerr << "DistributedServer::Configure: ERROR! Unable to configure "
                 "simulator: "
              << sim->getServerIp() << ":" << sim->getServerPort() << std::endl;
    return false;
  }

  for(auto& sensor : sensors) {
    // Actually configure the sensor to start streaming
    if(!sensor->configure()) {
      std::cerr << "DistributedServer::Configure: ERROR! Unable to configure "
                   "sensor type: "
                << sensor->config->type << " on port "
                << sensor->config->listen_port << " for server "
                << sim->getServerIp() << ":" << sim->getServerPort()
                << std::endl;
    }
    // Store all the port numbers so we know what we can use later when
    // dynamically creating sensors
    if (kServerReservedPorts.count(sensor->config->listen_port) > 0 and
        sensor->config->type != "ViewportCamera") {
      std::cerr << "DistributedServer::Configure: ERROR! Server port conflict "
                   "for sensor type: "
                << sensor->config->type << " on port "
                << sensor->config->listen_port << " for server "
                << sim->getServerIp() << ":" << sim->getServerPort()
                << std::endl;
    } else {
      kServerReservedPorts.insert(sensor->config->listen_port);
    }
  }
  return true;
}

bool DistributedServer::IsSampling() {
  return !sample_complete.load(std::memory_order_relaxed) ||
         sim->sampleInProgress(sensors);
}

void PrimaryDistributedServer::StateSensorCallback(DataFrame* frame) {
  // Grab the state data and signal that its available
  if (state_data_string != nullptr) {
    *state_data_string =
        static_cast<BinaryStateFrame*>(frame)->state_buffer.as_string();
  }
  state_sensor_data_updated.store(true, std::memory_order_relaxed);
}

bool PrimaryDistributedServer::Sample(std::string* state_data) {
  // Store the pointer for output in the callback
  state_data_string = state_data;

  // For primaries, always wait for the state data to come back
  bool success = sim->sampleAll(sensors);
  if (success) {
    while (!state_sensor_data_updated.load(std::memory_order_relaxed));
    state_sensor_data_updated = false;
  }
  sample_complete.store(true, std::memory_order_relaxed);

  return success;
}

bool PrimaryDistributedServer::Configure() {
  if(!DistributedServer::Configure()) {
    return false;
  }
  // Search for an existing state sensor that's streaming binary data
  bool bin_state_added = false;
  for (auto& sensor : sensors) {
    if (sensor->config->type == "BinaryState") {
      auto state_config = static_cast<StateConfig*>(sensor->config.get());
      if (state_config->send_binary_frame) {
        sensors.back()->sampleCallback =
            std::bind(&PrimaryDistributedServer::StateSensorCallback, this,
                      std::placeholders::_1);
        bin_state_added = true;
        break;
      }
    }
  }

  // If the sensor didn't exist, we have to add it to the primary so it can
  // distribute the state data to replicas
  if(!bin_state_added) {
    int listen_port = 8100;
    while(kServerReservedPorts.count(listen_port) > 0){
      listen_port++;
    }

    StateConfig s_config;
    s_config.server_ip = sim->getServerIp();
    s_config.server_port = sim->getServerPort();
    s_config.listen_port = listen_port;
    s_config.desired_tags = {"vehicle", "dynamic"};
    s_config.include_obb = false;
    s_config.debug_drawing = false;
    s_config.send_binary_frame = true;
    sensors.emplace_back(
        std::make_shared<Sensor>(std::make_unique<StateConfig>(s_config)));
    sensors.back()->sampleCallback =
        std::bind(&PrimaryDistributedServer::StateSensorCallback, this,
                  std::placeholders::_1);
    if(!sensors.back()->configure()) {
      std::cerr << "PrimaryDistributedServer::Configure: Unable to configure "
                   "binary state sensor!"
                << std::endl;
      return false;
    }

  }
  return true;
}

bool ReplicaDistributedServer::Sample(std::string* state_data) {
  // Replicas just get a state update
  nlohmann::json state_data_json;
  state_data_json["state_data_as_string"] = *state_data;
  bool success = sim->stateStepAll(sensors, state_data_json);
  sample_complete.store(true, std::memory_order_relaxed);
  return success;
}
