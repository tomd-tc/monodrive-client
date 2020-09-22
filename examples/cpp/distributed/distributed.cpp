#include <chrono>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <mutex>
#include <thread>

#include "Configuration.h"
#include "Sensor.h"
#include "Simulator.h"
#include "Stopwatch.h"
#include "DistributedServer.h"
#include "sensor_config.h"


// The shared state data object that the primary will populate and the replicas
// will use
std::string SHARED_STATE_DATA("");
// Set up a double buffering system so there's always a buffer available to 
// the primary server and we don't have to do a full copy
std::string SHARED_STATE_DATA_BUFFER("");
// A mutex to protect the read/write state of the SHARED_STATE_DATA
std::mutex STATE_DATA_MUTEX;
// Flag to stop the example from running
bool RUN_EXAMPLE=true;


void PrimaryThread(std::shared_ptr<PrimaryDistributedServer> server, Event* replicasReadyEvent) {
  uint64_t control_time = 0;
  int frame_count = 0;
  mono::precise_stopwatch primary_stopwatch;

  std::cout << "Primary thread starting..." << std::endl;
  auto egoControlServer = std::make_unique<Simulator>(server->GetSimulator()->getServerIp(), server->GetSimulator()->getServerPort());
  egoControlServer->connect();
  while (RUN_EXAMPLE) {
    auto control_start_time =
        primary_stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>();

    {
      std::lock_guard<std::mutex> lock(STATE_DATA_MUTEX);
      server->sample(&SHARED_STATE_DATA);
    }

    // apply controls
    EgoControlConfig ego_control_config;
    ego_control_config.forward_amount = 0.3f;
    ego_control_config.right_amount = 0.0f;
    ego_control_config.brake_amount = 0.0f;
    ego_control_config.drive_mode = 1;
    egoControlServer->sendCommandAsync(ego_control_config.message());

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
    Event* replicasReadyEvent) {
  uint64_t sample_time = 0;
  int frame_count = 0;
  mono::precise_stopwatch replica_stopwatch;
  std::string local_state_data("");

  std::cout << "Replica thread starting..." << std::endl;
  while (RUN_EXAMPLE) {
    // Cue the replicas and time them
    auto sample_start_time =
        replica_stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>();

    // Using a try_lock here because a scoped lock has too much overhead
    // Just keep  trying and breaking until we get it
    while (!STATE_DATA_MUTEX.try_lock());
    std::swap(SHARED_STATE_DATA, SHARED_STATE_DATA_BUFFER);
    STATE_DATA_MUTEX.unlock();

    // wait for primary
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

int main(int argc, char** argv) {
  /// The primary server needs a scenario file for closed loop mode
  Configuration primary_config(
      "examples/config/simulator_straightaway.json",
      "examples/config/weather.json",
      "examples/config/scenario_multi_vehicle_straightaway.json",
      "examples/cpp/distributed/primary_sensors.json");
  /// The replica servers just need to be forced into replay mode
  Configuration replica_lidar_config(
      "examples/config/simulator_straightaway.json",
      "examples/config/weather.json", 
      "examples/config/scenario.json",
      "examples/cpp/distributed/replica_lidar_sensors.json");
  Configuration replica_radar_config(
      "examples/config/simulator_straightaway.json",
      "examples/config/weather.json", 
      "examples/config/scenario.json",
      "examples/cpp/distributed/replica_radar_sensors.json");

  // Set up all the server
  auto primaryServer = std::make_shared<PrimaryDistributedServer>(
      primary_config, "127.0.0.1", 8999);
  std::vector<std::shared_ptr<ReplicaDistributedServer>> replicaServers = {
      std::make_shared<ReplicaDistributedServer>(replica_lidar_config,
                                                 "192.168.2.8", 8999),
      std::make_shared<ReplicaDistributedServer>(replica_lidar_config,
                                                 "192.168.86.41", 9000),
  };

  // Configure the primary first
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

  auto sig_handler = [](int signum) {
    RUN_EXAMPLE=false;
  };
  signal(SIGINT, sig_handler);

  // Kick off the orchestration threads
  Event* replicasReadyEvent = new Event(replicaServers.size());
  std::thread replicaThread(ReplicaThread, primaryServer, replicaServers, replicasReadyEvent);
  std::thread primaryThread(PrimaryThread, primaryServer, replicasReadyEvent);

  primaryThread.join();
  replicaThread.join();

  delete replicasReadyEvent;
  return 0;
}
