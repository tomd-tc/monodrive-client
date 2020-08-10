#include <chrono>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "Configuration.h"
#include "Sensor.h"
#include "Simulator.h"
#include "Stopwatch.h"
#include "distributed_server.h"
#include "sensor_config.h"

namespace ds = distributed_server;

int main(int argc, char** argv) {
  // Set up all the server
  ds::DistributedServer primary_server =
      ds::DistributedServer("127.0.0.1", 8999, ds::kServerType::PRIMARY);
  std::vector<ds::DistributedServer> replica_servers = {
      ds::DistributedServer("192.168.2.3", 8999, ds::kServerType::REPLICA),
      ds::DistributedServer("192.168.2.3", 9000, ds::kServerType::REPLICA)};

  // Configure the senors for each server
  if (!primary_server.Configure(
          {ds::kSensorType::VIEWPORT, ds::kSensorType::BINARY_STATE}) ||
      !replica_servers[0].Configure(
          {ds::kSensorType::VIEWPORT, ds::kSensorType::RADAR}) ||
      !replica_servers[1].Configure(
          {ds::kSensorType::VIEWPORT, ds::kSensorType::LIDAR})) {
    return -1;
  }

  // Bench marking stuff
  int frame_count = 0;
  uint64_t control_time = 0;
  uint64_t sample_time = 0;
  mono::precise_stopwatch stopwatch;
  // Shared data between servers
  nlohmann::json state_data;
  while (true) {
    // Cue the primary and time it
    auto control_start_time =
        stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>();
    primary_server.Sample(&state_data);
    control_time +=
        stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>() -
        control_start_time;

    // Cue the replicas and time them
    auto sample_start_time =
        stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>();
    for (auto server : replica_servers) {
      server.Sample(&state_data);
    }
    sample_time +=
        stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>() -
        sample_start_time;

    // Spit out FPS data every 10 frames
    if ((++frame_count) % 10 == 0) {
      std::cout << "Control Avg: " << (control_time / frame_count) / 1e6
                << std::endl;
      std::cout << "Sample Avg: " << (sample_time / frame_count) / 1e6
                << std::endl;
      std::cout << "FPS: " << frame_count / ((control_time + sample_time) / 1e6)
                << std::endl;
      frame_count = 0;
      control_time = 0;
      sample_time = 0;
    }
  }

  return 0;
}
