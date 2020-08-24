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


/// Global static for this class so nobody has any ports that clash
static std::set<int> kServerReservedPorts;

class Event
{
public:
    inline Event(int count = 1) :
        eventCount(count), 
        eventNumber(count) 
    {}

    inline void wait() const {
        std::unique_lock< std::mutex > lock(mutex);
        condition.wait(lock,[&]()->bool{ return eventCount > 0; });
    }

    inline bool notify() {
        bool signalled;
        mutex.lock();
        eventCount--;
        signalled = eventCount <= 0;
        mutex.unlock();

        if (eventCount <= 0) {
            condition.notify_all();
            reset();
        }
        return signalled;
    }
	
    inline void reset() {
        mutex.lock();
        eventCount = eventNumber;
        mutex.unlock();
    }

    inline bool hasPending() const { return eventCount > 0; }

private:
    int eventCount;
    int eventNumber;
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
                      const int& port,
                      const int& simulationMode);
                      
    virtual ~DistributedServer() {
        delete sampleComplete;
    }
    
    virtual bool loadSensors();
    /// @brief Configure the specified sensors for the server
    /// @return true if all sensors were successfully configured
    virtual bool configure();
    /// @brief Sample the sensors on this server. Pure virtual.
    /// @param stateData - The state data to process during the sample
    /// @return true if the sample was successful
    virtual bool sample(std::string* stateData) = 0;
    /// @brief Determine if this server is still sampling sensors
    /// @return true if a sample is still in progress
    virtual bool isSampling();
    /// @brief Send command to server and block until response
    virtual bool sendCommand(ApiMessage message, nlohmann::json* response=nullptr);
    /// @brief Send command to server and return immediately
    virtual bool sendCommandAsync(ApiMessage message, nlohmann::json* response=nullptr);


    /// @brief Event that triggers when a sample send is completed
    Event* sampleComplete = nullptr;

    /// The array of sensors that is configured for this server
    std::vector<std::shared_ptr<Sensor>> sensors;

 protected:
    virtual std::function<void(DataFrame*)> setupCallback(std::shared_ptr<Sensor> sensor);

    /// A pointer to the server object
    Simulator* sim = nullptr;
    /// @brief The set of configuration items for this server
    Configuration serverConfig;
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
        : DistributedServer(config, ip_address, port, 0) {}
    /// @brief String that will be populated with the state data when this
    /// function returns. Must be instantiated.
    /// @param stateData - The state data to process during the sample
    /// @return true if the sample call was successful
    bool sample(std::string* stateData) final;
    /// @brief Handle adding a predefined sensor to the server
    /// @return true if the configuration was successful
    bool configure() override final;

protected:
    virtual std::function<void(DataFrame*)> setupCallback(std::shared_ptr<Sensor> sensor) override;
private:
    std::string* stateDataString = nullptr;
};

/// @class Class for managing a remote Simulator that collects state data
class ReplicaDistributedServer : public DistributedServer {
 public:
  /// @brief Constructor
  /// @param config - The set of configurations to use with the server
  /// @param ipAddress - The IP address to the server
  /// @param port - The port number for server command communication
  ReplicaDistributedServer(const Configuration& config,
                           const std::string& ipAddress, const int& port)
      : DistributedServer(config, ipAddress, port, 2)
      {}
  /// @brief String of state data to send to the replica to update the physics
  /// state of the server 
  /// @param stateData - The state data to process during the sample
  /// @return true if the sample call was successful
  bool sample(std::string* stateData) final;
 private:
};
