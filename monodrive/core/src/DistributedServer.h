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

class Event
{
public:
    inline Event(int count = 1) :
        event_count(count), 
        event_number(count) 
    {}

    inline void Wait() const {
        std::unique_lock< std::mutex > lock(mutex);
        condition.wait(lock,[&]()->bool{ return event_count > 0; });
    }

    inline bool Notify() {
        bool signalled;
        mutex.lock();
        event_count--;
        signalled = event_count <= 0;
        mutex.unlock();

        if (event_count <= 0) {
            condition.notify_all();
            Reset();
        }
        return signalled;
    }
	
    inline void Reset() {
        mutex.lock();
        event_count = event_number;
        mutex.unlock();
    }

    inline bool HasPending() const { return event_count > 0; }

private:
    int event_count;
    int event_number;
    mutable std::mutex mutex;
    mutable std::condition_variable condition;
};

/// @class Base class for managing a remote Simulator instance
class DistributedServer {
public:
    /// @brief Constructor
    /// @param config - The set of configurations to use with the server
    /// @param ip_address - The IP address to the server
    /// @param port - The port number for server command communication
    DistributedServer(const Configuration& config,
                      const std::string& ip_address,
                      const int& port);
                      
    virtual ~DistributedServer() {
        delete sample_complete;
    }
    
    virtual bool LoadSensors();
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

    Event* sample_complete = nullptr;

    /// The array of sensors that is configured for this server
    std::vector<std::shared_ptr<Sensor>> sensors;

 protected:
    virtual std::function<void(DataFrame*)> SetupCallback(std::shared_ptr<Sensor> sensor);

    /// A pointer to the server object
    Simulator* sim = nullptr;
    /// @brief The set of configuration items for this server
    Configuration server_config;
    /// @brief Flag that is set when a sample send is completed
//    std::atomic<bool> sample_complete;
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

protected:
    virtual std::function<void(DataFrame*)> SetupCallback(std::shared_ptr<Sensor> sensor) override;
private:
    std::string* state_data_string = nullptr;
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
      : DistributedServer(config, ip_address, port)
      {}
  /// @brief String of state data to send to the replica to update the physics
  /// state of the server 
  /// @param state_data - The state data to process during the sample
  /// @return true if the sample call was successful
  bool Sample(std::string* state_data) final;
 private:
};
}