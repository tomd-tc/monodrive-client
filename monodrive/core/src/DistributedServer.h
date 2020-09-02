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
static std::set<std::string> kServerReservedPorts;

/// @class event class that uses std::condition_variable to 
/// implement non-busy wait and count based notifications
class Event
{
public:
    /// @brief constructor
    /// @param count - the number of times notify needs to get called in order to trigger event
    inline Event(int count = 0) :
        eventCount(count), 
        eventNumber(count) 
    {}

    inline ~Event() {
        eventCount = 0;
        condition.notify_all();
    }

    /// @brief blocks the current thread until eventCount is zero
    inline void wait() const {
        std::unique_lock< std::mutex > lock(mutex);
        condition.wait(lock,[&]()->bool{ return eventCount > 0; });
    }

    /// @brief decrements eventCount. If eventCount becomes zero, the event is
    /// triggered and any threads waiting on the condition get signaled.
    /// When the event is triggered, eventCount is reset to the original value
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
	
    /// @brief resets the event to initial state
    inline void reset(int newCount = -1) {
        mutex.lock();
        if (newCount > 0) {
            eventNumber = newCount;
        }
        eventCount = eventNumber;
        mutex.unlock();
    }

    /// @brief returns true if event has pending items before it triggers
    inline bool hasPending() const { return eventCount > 0; }

private:
    /// the number of times notify needs to be called to trigger the event
    int eventCount;
    /// the value used to reset the event to initial state
    int eventNumber;
    /// mutex to synchronize access to state and to wait on condition variable
    mutable std::mutex mutex;
    /// the condition variable used for non-busy waits
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
                      
    /// @brief destructor
    virtual ~DistributedServer() {
        delete sampleComplete;
    }
    
    /// @brief create the specified sensors for the server
    /// @return true if all sensors were created successfully
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
    /// @brief set up the sensor callback for the given sensor.
    /// @param sensor - the sensor to set up the callback for
    /// @return the callback for the sensor that calls sampleComplete->notify() 
    /// to signal sample completion, and to forward the sensor data to any
    /// callback registered with the sensor
    virtual std::function<void(DataFrame*)> setupCallback(std::shared_ptr<Sensor> sensor);
    /// @brief returns the number of streaming sensors in the sensors array
    int getStreamingSensorsCount();
    /// @brief Create the UID for the port for insertion into the global set
    /// @param portNumber - The port to use
    /// @return string representing the UID for the port
    std::string createPortKey(const int& portNumber);

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
    std::shared_ptr<Sensor> binaryStateSensor;
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
