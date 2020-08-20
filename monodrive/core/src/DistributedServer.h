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

/// Servers are either the primary controller or the replica runners
enum kServerType {
    PRIMARY = 0,
    REPLICA
};

/// Available sensor types for distributed server testing
enum kSensorType {
    BINARY_STATE = 0,
    STATE,
    VIEWPORT,
    RADAR,
    LIDAR,
    IMU,
    WAYPOINT
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

/// @class A class for managing benchmarking on distributed servers
class DistributedServer {
public:
    /// @brief Constructor
    /// @param ip_address - The IP address to the server
    /// @param port - The port number for server command communication
    /// @param server_type - Either the primary or replica server
    DistributedServer(const std::string& ip_address, const int& port,
                    const kServerType& server_type);
    /// @brief Copy constructor
    /// @param rhs - The server to copy
    DistributedServer(const DistributedServer& rhs);
    /// @brief Destructor
    ~DistributedServer();
    /// @brief Assignment overload
    /// @param rhs - The server to copy
    /// @return The server to copy to
    DistributedServer& operator=(const DistributedServer& rhs) = default;
    /// @brief Configure the specified sensors for the server
    /// @param sensor_types - A vector of kSensorTypes to run on this server
    /// @return true if all sensors were successfully configured
    bool Configure(const std::vector<kSensorType>& sensor_types);
    bool Configure();
    /// @brief Sample the sensors on this server
    /// @param state_data - If this is a primary server this is populated with the
    /// current state data from the primary server. If this is a replica then the
    /// state data will be sent to server for update.
    /// return true if the sampling was successful
    /// @param async - If true, then this call will be non-blocking. Default is
    /// false.
    bool Sample(/*std::string* state_data,*/ bool async=false);
    /// @brief Determine if this server is still sampling sensors
    /// @return true if a sample is still in progress
    bool IsSampling();

    /// @brief Handle adding a predefined sensor to the server
    /// @param sensor_type - The sensor to add
    bool AddSensor(kSensorType sensor_type, std::function<void(DataFrame*)> callback = nullptr);

private:
    /// @brief The callback that will handle incoming state data for
    /// primary servers.
    /// @param frame - The incoming state data frame
    //void StateSensorCallback(DataFrame* frame);
    /// @brief The main worker thread for triggering sensor samples
    void SampleThread();

    /// The array of sensors that is configured for this server
    std::vector<std::shared_ptr<Sensor>> sensors;
    /// A pointer to the server object
    Simulator* sim = nullptr;
    /// The current type of server
    kServerType server_type;
    /// Flag to communicate if state data has been received or not
//    bool state_sensor_data_updated = false;
    /// The pointer to the current set of state data
//    std::string* state_data_string = nullptr;
    /// Condition variable to signal that a sample should occur
    std::condition_variable sample_trigger;
    /// Mutex for the sample thread's trigger
    std::mutex sample_mutex;
    /// True if the server is ready for another sample
    bool ready_to_sample{false};
    /// True if the current sample has been completed
    bool sample_complete{false};
    /// Mutex to wait for a non-async sample to complete
    std::mutex sample_complete_mutex;
    /// Thread that performs a sample on this server
    std::thread sample_thread;
    /// Condition that we are currently connected to a server
    bool connected{true};
 };
}