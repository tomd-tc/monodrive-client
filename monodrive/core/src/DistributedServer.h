#pragma once

#include <vector>
#include <set>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "Configuration.h"
#include "Sensor.h"
#include "Simulator.h"
#include "DataFrame.h"

namespace distributed_server {

/// Global static for this class so nobody has any ports that clash
static std::set<int> kServerReservedPorts;

/// Available sensor types for distributed server testing
enum kSensorType {
    BINARY_STATE = 0,
    STATE,
    VIEWPORT,
    RADAR,
    LIDAR
};

/// The primary server needs a scenario file for closed loop mode
static const Configuration kPrimaryConfig(
    "examples/config/simulator_straightaway.json",
    "examples/config/weather.json",
    "examples/config/scenario_multi_vehicle_straightaway.json");
/// The replica servers just need to be forced into replay mode
static const Configuration kReplicaConfig(
    "examples/config/simulator_straightaway_replay.json",
    "examples/config/weather.json", 
    "examples/config/scenario.json");

/// @class Base class for managing a remote Simulator instance
class DistributedServer {
public:
 /// @brief Constructor
 /// @param config - The set of configurations to use with the server
 /// @param ip_address - The IP address to the server
 /// @param port - The port number for server command communication
 /// @param server_type - Either the primary or replica server
 DistributedServer::DistributedServer(const Configuration& config,
                                      const std::string& ip_address,
                                      const int& port);
 /// @brief Configure the specified sensors for the server
 /// @param sensor_types - A vector of kSensorTypes to run on this server
 /// @return true if all sensors were successfully configured
 virtual bool Configure(const std::vector<kSensorType>& sensor_types);
 /// @brief Sample the sensors on this server. Pure virtual.
 /// @param state_data - The state data to process during the sample
 virtual bool Sample(std::string* state_data) = 0;
 /// @brief Determine if this server is still sampling sensors
 /// @return true if a sample is still in progress
 virtual bool IsSampling();

 protected:
  /// @brief Handle adding a predefined sensor to the server
  /// @param sensor_type - The sensor to add
  virtual bool AddSensor(kSensorType sensor_type);
  /// A pointer to the server object
  Simulator* sim = nullptr;
  /// The array of sensors that is configured for this server
  std::vector<std::shared_ptr<Sensor>> sensors;
  /// @brief The set of configuration items for this server
  Configuration server_config;
  /// @brief Flag that is set when a sample send is completed
  std::atomic<bool> sample_complete;
 };

/// @class Class for managing a remote Simulator that collects state data
class PrimaryDistributedServer : public DistributedServer {
 public:
  /// @brief Constructor
  /// @param ip_address - The IP address to the server
  /// @param port - The port number for server command communication
  /// @param server_type - Either the primary or replica server
  PrimaryDistributedServer(const std::string& ip_address, const int& port)
      : DistributedServer(kPrimaryConfig, ip_address, port) {}
  /// @brief String that will be populated with the state data when this
  /// function returns. Must be instantiated.
  /// @param state_data - The state data to process during the sample
  /// @return true if the sample call was successful
  bool Sample(std::string* state_data) final;
 private:
  /// @brief Handle adding a predefined sensor to the server
  /// @param sensor_type - The sensor to add
  /// @return true if the sensor adds were successful
  bool AddSensor(kSensorType sensor_type) override final;
  /// @brief The callback that will handle incoming state data for
  /// primary servers.
  /// @param frame - The incoming state data frame
  void StateSensorCallback(DataFrame* frame);
  /// The pointer to the current set of state data
  std::string* state_data_string = nullptr;
  /// Flag to communicate if state data has been received or not
  std::atomic<bool> state_sensor_data_updated = false;
};

/// @class Class for managing a remote Simulator that collects state data
class ReplicaDistributedServer : public DistributedServer {
 public:
  /// @brief Constructor
  /// @param ip_address - The IP address to the server
  /// @param port - The port number for server command communication
  /// @param server_type - Either the primary or replica server
  ReplicaDistributedServer(const std::string& ip_address, const int& port)
      : DistributedServer(kReplicaConfig, ip_address, port) {}
  /// @brief String of state data to send to the replica to update the physics
  /// state of the server 
  /// @param state_data - The state data to process during the sample
  /// @return true if the sample call was successful
  bool Sample(std::string* state_data) final;
 private:
};
}