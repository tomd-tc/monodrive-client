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

namespace ds = distributed_server;

// The shared state data object that the primary will populate and the replicas
// will use
std::string SHARED_STATE_DATA("");
// Set up a double buffering system so there's always a buffer available to 
// the primary server and we don't have to do a full copy
std::string SHARED_STATE_DATA_BUFFER("");
// A mutex to protect the read/write state of the SHARED_STATE_DATA
std::mutex STATE_DATA_MUTEX;

void PrimaryThread(std::shared_ptr<ds::DistributedServer> server) {
  uint64_t control_time = 0;
  int frame_count = 0;
  mono::precise_stopwatch primary_stopwatch;

  std::cout << "Primary thread starting..." << std::endl;
  while (true) {
    auto control_start_time =
        primary_stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>();

    {
      std::lock_guard<std::mutex> lock(STATE_DATA_MUTEX);
      server->Sample(&SHARED_STATE_DATA);
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
  }
}

void ReplicaThread(
    std::vector<std::shared_ptr<ds::DistributedServer>> servers) {
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

    if (SHARED_STATE_DATA_BUFFER == "") {
      continue;
    }

    for (auto& server : servers) {
      server->Sample(&SHARED_STATE_DATA_BUFFER, true);
    }

    for (auto& server : servers) {
      while (server->IsSampling());
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
  // Set up all the server
  auto primary_server =
      std::make_shared<ds::DistributedServer>("127.0.0.1", 8999,
                                              ds::kServerType::PRIMARY);
  std::vector<std::shared_ptr<ds::DistributedServer>> replica_servers = {
      std::make_shared<ds::DistributedServer>("192.168.2.3", 8999,
                                              ds::kServerType::REPLICA),
      std::make_shared<ds::DistributedServer>("192.168.2.3", 9000,
                                              ds::kServerType::REPLICA)
  };

  // Configure the senors for each server
  if (!primary_server->Configure(
          {ds::kSensorType::VIEWPORT, ds::kSensorType::BINARY_STATE}) 
          ||
      !replica_servers[0]->Configure(
          {ds::kSensorType::VIEWPORT, ds::kSensorType::RADAR})
          ||
      !replica_servers[1]->Configure(
          {ds::kSensorType::VIEWPORT, ds::kSensorType::LIDAR})
          ) {
    return -1;
  }

  // Bench marking stuff
  std::thread primary_thread(PrimaryThread, primary_server);
  std::thread replica_thread(ReplicaThread, replica_servers);

  primary_thread.join();
  replica_thread.join();
  return 0;
}
