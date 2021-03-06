#include <functional>

#include "DistributedServer.h"
#include "Util.h"


DistributedServer::DistributedServer(const Configuration& config,
                                     const std::string& ipAddress,
                                     const int& port,
                                     const int& simulationMode)
    : serverConfig(config) {
  // Make sure to copy the desired IP and port to the configurations since we
  // may be getting a resused config file
  serverConfig.simulator["server_ip"] = ipAddress;
  serverConfig.simulator["server_port"] = port;
  serverConfig.simulator["simulation_mode"] = simulationMode;
  // Set all the sensors to this server's IP address so we don't use a value
  // from the shared config files.
  for(auto& sensor : serverConfig.sensorsConfig) {
    sensor["server_ip"] = ipAddress;
    sensor["server_port"] = port;
  }

  // Go ahead and grab an instance of the server before configuring
  sim = &Simulator::getInstance(serverConfig, ipAddress, port);

  // All server ports are technically reserved as well
  kServerReservedPorts.insert(createPortKey(port));
}

std::string DistributedServer::createPortKey(const int& portNumber) {
  if (sim != nullptr) {
    return sim->getServerPort() + ":" + std::to_string(portNumber);
  } else {
    return std::to_string(portNumber);
  }
}

bool DistributedServer::loadSensors() {
  sensors.clear();

  // Read in all the sensor configurations for this server
  if (!serverConfig.loadSensors(sensors)) {
    std::cerr << "DistributedServer::configure: ERROR! Unable to configure one "
                 "or more sensors on server: "
              << sim->getServerIp() << ":" << sim->getServerPort() << std::endl;
    return false;
  }

  return true;
}

bool DistributedServer::configure(){
  if (sensors.size() == 0 && !loadSensors()) {
    return false;
  }

  // Configure the simulator
  if (!sim->configure()) {
    std::cerr << "DistributedServer::configure: ERROR! Unable to configure "
                 "simulator: "
              << sim->getServerIp() << ":" << sim->getServerPort() << std::endl;
    return false;
  }

  for(auto& sensor : sensors) {
    // Actually configure the sensor to start streaming
    if(!sensor->configure()) {
      std::cerr << "DistributedServer::configure: ERROR! Unable to configure "
                   "sensor type: "
                << sensor->config->type << " on port "
                << sensor->config->listen_port << " for server "
                << sim->getServerIp() << ":" << sim->getServerPort()
                << std::endl;
    }
    // Store all the port numbers so we know what we can use later when
    // dynamically creating sensors
    if (kServerReservedPorts.count(createPortKey(sensor->config->listen_port)) >
            0 and
        sensor->config->type != "ViewportCamera") {
      std::cerr << "DistributedServer::configure: ERROR! Server port conflict "
                   "for sensor type: "
                << sensor->config->type << " on port "
                << sensor->config->listen_port << " for server "
                << sim->getServerIp() << ":" << sim->getServerPort()
                << std::endl;
    } else {
      kServerReservedPorts.insert(createPortKey(sensor->config->listen_port));
    }
  }
  return true;
}

bool DistributedServer::isSampling() {
    return !sampleComplete.load(std::memory_order_relaxed) ||
         sim->sampleInProgress(sensors);
}

bool DistributedServer::sendCommand(ApiMessage message, nlohmann::json* response) {
  return sim->sendCommand(message, response);
}

bool DistributedServer::sendCommandAsync(ApiMessage message, nlohmann::json* response) {
  return sim->sendCommandAsync(message, response);
}

int DistributedServer::getStreamingSensorsCount() {
  int sensorCount = 0;
  for (auto& sensor : sensors) {
    if (sensor->config->type == "ViewportCamera" or
        !sensor->config->enable_streaming)
      continue;

    sensorCount++;
  }
  return sensorCount;
}

bool PrimaryDistributedServer::sample(std::string* stateData) {
  // Store the pointer for output in the callback
  stateDataString = stateData;
  // For primaries, always wait for the state data to come back
  bool success = sim->sampleAll(sensors);
  if (success) {
    while(!stateSensorDataUpdated.load(std::memory_order_relaxed));
    stateSensorDataUpdated = false;
  }
  sampleComplete.store(true, std::memory_order_relaxed);

  return success;
}

bool PrimaryDistributedServer::configure() {
  if(!DistributedServer::configure()) {
    return false;
  }

  // If the sensor didn't exist, we have to add it to the primary so it can
  // distribute the state data to replicas
    int listenPort = 8122;
    while(kServerReservedPorts.count(createPortKey(listenPort)) > 0){
      listenPort++;
    }

    StateConfig sConfig;
    sConfig.server_ip = sim->getServerIp();
    sConfig.server_port = sim->getServerPort();
    sConfig.listen_port = listenPort;
    sConfig.desired_tags = {"vehicle", "dynamic"};
    sConfig.include_obb = false;
    sConfig.debug_drawing = false;
    sConfig.enable_streaming = true;
    auto binaryStateSensor = std::make_shared<Sensor>(std::make_unique<StateConfig>(sConfig), false);
    binaryStateSensor->parseBinaryData = false;
    binaryStateSensor->sampleCallback = [this](DataFrame* frame) {
            if (stateDataString != nullptr) {
              *stateDataString = static_cast<BinaryDataFrame*>(frame)->data_frame.as_string();
            }            
            stateSensorDataUpdated.store(true, std::memory_order_relaxed);
    };
    sensors.emplace_back(binaryStateSensor);

    if(!binaryStateSensor->configure()) {
      std::cerr << "PrimaryDistributedServer::Configure: Unable to configure "
                   "binary state sensor!"
                << std::endl;
      return false;
    }

  return true;
}

bool ReplicaDistributedServer::sample(std::string* state_data) {
  // Replicas just get a state update
  nlohmann::json state_data_json;
  state_data_json["state_data_as_string"] = *state_data;
  bool success = sim->stateStepAll(sensors, state_data_json);
  sampleComplete.store(true, std::memory_order_relaxed);
  return success;
}
