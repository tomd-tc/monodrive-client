#include <iostream> // std::cout
#include <chrono>   // std::chrono
#include <thread>   // std::thread
#include <memory>
#include <vector>

//monoDrive Includes
#include "Simulator.h"
#include "Configuration.h" // holder for sensor, simulator, scenario, weather, and vehicle configurations
#include "Sensor.h"
#include "sensor_config.h"
#include "Stopwatch.h"
#include <future>

#include "ros/ros.h"
#include "ros/package.h"
#include "DistributedServer.h"
#include "MessageFactory.h"

#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

// The shared state data object that the primary will populate and the replicas
// will use
std::string SHARED_STATE_DATA("");
// Set up a double buffering system so there's always a buffer available to 
// the primary server and we don't have to do a full copy
std::string SHARED_STATE_DATA_BUFFER("");
// A mutex to protect the read/write state of the SHARED_STATE_DATA
std::mutex STATE_DATA_MUTEX;


void rosLoop(float fps)
{

  ros::Rate rate(fps);

  while (ros::ok())
  {
    // Sample the sensors
    rate.sleep();
  }
}

void PrimaryThread(std::shared_ptr<PrimaryDistributedServer> server, std::shared_ptr<Event> replicasReadyEvent) {
  uint64_t control_time = 0;
  int frame_count = 0;
  mono::precise_stopwatch primary_stopwatch;

  std::cout << "Primary thread starting..." << std::endl;
  while (true) {
    auto control_start_time =
        primary_stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>();

    {
      std::lock_guard<std::mutex> lock(STATE_DATA_MUTEX);
      server->sample(&SHARED_STATE_DATA);
    }

    control_time +=
        primary_stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>() -
        control_start_time;

    // Spit out FPS about every second
    if ((++frame_count) % 100 == 0) {
      std::cout << "=== Control Avg: " << (control_time / frame_count) / 1e6
                << std::endl;
      std::cout << "FPS: " << frame_count / (control_time / 1e6) << std::endl;
      frame_count = 0;
      control_time = 0;
    }

    // wait for replicas to finish
    replicasReadyEvent->wait();
  }
}

void ReplicaThread(
  std::shared_ptr<PrimaryDistributedServer> primary,
    std::vector<std::shared_ptr<ReplicaDistributedServer>> servers, 
    std::shared_ptr<Event> replicasReadyEvent) {
  uint64_t sample_time = 0;
  int frame_count = 0;
  mono::precise_stopwatch replica_stopwatch;
  std::string local_state_data("");

  std::cout << "Replica thread starting..." << std::endl;
  while (true) {
    // Cue the replicas and time them
    auto sample_start_time =
        replica_stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>();

    // Using a try_lock here because a scoped lock has too much overhead
    // Just keep  trying and breaking until we get it
    while (!STATE_DATA_MUTEX.try_lock());
    std::swap(SHARED_STATE_DATA, SHARED_STATE_DATA_BUFFER);
    STATE_DATA_MUTEX.unlock();

    primary->sampleComplete->wait();

    if (SHARED_STATE_DATA_BUFFER == "") {
      continue;
    }

    for (auto& server : servers) {
      server->sample(&SHARED_STATE_DATA_BUFFER);
    }

    for (auto& server : servers) {
      while (server->isSampling());

      // signal complete
      replicasReadyEvent->notify();
    }

    sample_time +=
        replica_stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>() -
        sample_start_time;

    // Spit out FPS about every second
    if ((++frame_count) % 30 == 0) {
      std::cout << "=== Sample Avg: " << (sample_time / frame_count) / 1e6
                << std::endl;
      std::cout << "FPS: " << frame_count / ((sample_time) / 1e6)
                << std::endl;
      frame_count = 0;
      sample_time = 0;
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_follower");
  ros::NodeHandle n;

/// The primary server needs a scenario file for closed loop mode
  Configuration primary_config(
      "examples/cpp/distributed/simulator_straightaway.json",
      "examples/config/weather.json",
      "examples/config/scenario_multi_vehicle_straightaway.json",
      "examples/ros/src/distributed/config/primary_sensors.json");
  /// The replica servers just need to be forced into replay mode
  Configuration replica_lidar_config(
      "examples/cpp/distributed/simulator_straightaway_replay.json",
      "examples/config/weather.json", 
      "examples/config/scenario.json",
      "examples/cpp/distributed/replica_lidar_sensors.json");
  Configuration replica_radar_config(
      "examples/cpp/distributed/simulator_straightaway_replay.json",
      "examples/config/weather.json", 
      "examples/config/scenario.json",
      "examples/cpp/distributed/replica_radar_sensors.json");

  // Set up all the server
  auto primaryServer = std::make_shared<PrimaryDistributedServer>(
      primary_config, "127.0.0.1", 8999);
  std::vector<std::shared_ptr<ReplicaDistributedServer>> replicaServers = {
      std::make_shared<ReplicaDistributedServer>(replica_radar_config,
                                                     "192.168.86.41", 8999),
      std::make_shared<ReplicaDistributedServer>(replica_lidar_config,
                                                     "192.168.86.41", 9000),
  };

  primaryServer->loadSensors();

  for (auto& sensor : primaryServer->sensors) {
    if (sensor->config->type == "State") {
      std::shared_ptr<ros::NodeHandle> node_handle_state = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
      ros::Publisher pub_state = node_handle_state->advertise<monodrive_msgs::StateSensor>("/monodrive/state", 1);
      sensor->sampleCallback = [pub_state](DataFrame *frame) {
        StateFrame data = *static_cast<StateFrame*>(frame);
        monodrive_msgs::StateSensor msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
        pub_state.publish(msg);
      };
    } 
    else if (sensor->config->type == "IMU") {
      std::shared_ptr<ros::NodeHandle> node_handle_imu = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
      ros::Publisher pub_imu = node_handle_imu->advertise<sensor_msgs::Imu>("/monodrive/imu", 1);
      sensor->sampleCallback = [pub_imu](DataFrame *frame) {
        ImuFrame data = *static_cast<ImuFrame*>(frame);
        sensor_msgs::Imu msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
        pub_imu.publish(msg);
      };
    }
    else if (sensor->config->type == "Waypoint") {
      std::shared_ptr<ros::NodeHandle> node_handle_waypoint = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
      ros::Publisher pub_waypoint = node_handle_waypoint->advertise<monodrive_msgs::WaypointSensor>("/monodrive/waypoint", 1);
      sensor->sampleCallback = [pub_waypoint](DataFrame *frame) {
        WaypointFrame data = *static_cast<WaypointFrame*>(frame);
        monodrive_msgs::WaypointSensor msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
        pub_waypoint.publish(msg);
      };
    }
  }

  // Configure the primary 
  if (!primaryServer->configure()){
    std::cerr << "Unable to configure primary server!" << std::endl;
    return -1;
  }
  // 
  for(auto& server : replicaServers) {
    if(!server->configure()){
      std::cout << "Unable to configure replica server!" << std::endl;
      return -1;
    }
  }

  // Bench marking stuff
  auto replicasReadyEvent = std::shared_ptr<Event>(new Event(replicaServers.size()));
  std::thread replicaThread(ReplicaThread, primaryServer, replicaServers, replicasReadyEvent);
  std::thread primaryThread(PrimaryThread, primaryServer, replicasReadyEvent);

  float fps = 100.f;
  rosLoop(fps);

  primaryThread.join();
  replicaThread.join();

  return 0;
}
