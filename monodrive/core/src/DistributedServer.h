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

/// @class Base class for managing a remote Simulator instance
class DistributedServer {
public:
 /// @brief Constructor
 /// @param config - The set of configurations to use with the server
 /// @param ip_address - The IP address to the server
 /// @param port - The port number for server command communication
 DistributedServer::DistributedServer(const Configuration& config,
                                      const std::string& ip_address,
                                      const int& port);
 /// @brief Configure the specified sensors for the server
 /// @return true if all sensors were successfully configured
 virtual bool Configure();
 /// @brief Sample the sensors on this server. Pure virtual.
 /// @param state_data - The state data to process during the sample
 /// @return true if the sample was successful
 virtual bool Sample(std::string* state_data) = 0;
 /// @brief Determine if this server is still sampling sensors
 /// @return true if a sample is still in progress
 virtual bool IsSampling();

 protected:
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
  /// @param config - The set of configurations to use with the server
  /// @param ip_address - The IP address to the server
  /// @param port - The port number for server command communication
  PrimaryDistributedServer(const Configuration& config,
                           const std::string& ip_address, const int& port)
      : DistributedServer(config, ip_address, port) {}
  /// @brief String that will be populated with the state data when this
  /// function returns. Must be instantiated.
  /// @param state_data - The state data to process during the sample
  /// @return true if the sample call was successful
  bool Sample(std::string* state_data) final;
  /// @brief Handle adding a predefined sensor to the server
  /// @return true if the configuration was successful
  bool Configure() override final;
 private:
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
  /// @param config - The set of configurations to use with the server
  /// @param ip_address - The IP address to the server
  /// @param port - The port number for server command communication
  ReplicaDistributedServer(const Configuration& config,
                           const std::string& ip_address, const int& port)
      : DistributedServer(config, ip_address, port) {}
  /// @brief String of state data to send to the replica to update the physics
  /// state of the server 
  /// @param state_data - The state data to process during the sample
  /// @return true if the sample call was successful
  bool Sample(std::string* state_data) final;
 private:
};
}