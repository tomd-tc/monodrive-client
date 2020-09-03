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
// Flag to tell all threads if we are still running
std::atomic<bool> ROS_RUNNING{true};


void rosLoop(float fps) {
  ros::Rate rate(fps);

  while (ros::ok())
  {
    // Sample the sensors
    rate.sleep();
  }

  ROS_RUNNING.store(false, std::memory_order_relaxed);
}

void PrimaryThread(std::shared_ptr<PrimaryDistributedServer> server) {
  uint64_t control_time = 0;
  int frame_count = 0;
  mono::precise_stopwatch primary_stopwatch;

  std::cout << "Primary thread starting..." << std::endl;
  while (ROS_RUNNING) {
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
    server->sendCommand(ego_control_config.message());

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
  std::shared_ptr<PrimaryDistributedServer> primary,
    std::vector<std::shared_ptr<ReplicaDistributedServer>> servers) {
  uint64_t sample_time = 0;
  int frame_count = 0;
  mono::precise_stopwatch replica_stopwatch;
  std::string local_state_data("");

  std::cout << "Replica thread starting..." << std::endl;
  while (ROS_RUNNING) {
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
      server->sample(&SHARED_STATE_DATA_BUFFER);
    }

    for (auto& server : servers) {
      while (server->isSampling());
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
  ros::init(argc, argv, "distributed");
  ros::NodeHandle n;

  /// The primary server needs a scenario file for closed loop mode
  Configuration primary_config(
      "examples/config/simulator_straightaway.json",
      "examples/config/weather.json",
      "examples/config/scenario_multi_vehicle_straightaway.json",
      "examples/ros/src/distributed/config/primary_sensors.json");
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
      primary_config, "10.100.11.151", 8999);
  std::vector<std::shared_ptr<ReplicaDistributedServer>> replicaServers = {
      // machine 1 instance 1
      std::make_shared<ReplicaDistributedServer>(replica_lidar_config,
                                                 "10.100.11.154", 8998),
      // machine 1 instance 2
      std::make_shared<ReplicaDistributedServer>(replica_radar_config,
                                                 "10.100.11.154", 8999),
      // machine 2 instance 3
      std::make_shared<ReplicaDistributedServer>(replica_radar_config,
                                                 "10.100.11.160", 8999),
      // machine 2 instance 4
      std::make_shared<ReplicaDistributedServer>(replica_radar_config,
                                                 "10.100.11.160", 9000),
      // machine 3 instance 5
      std::make_shared<ReplicaDistributedServer>(replica_radar_config,
                                                 "10.100.11.163", 8999),
      // machine 3 instance 6
      std::make_shared<ReplicaDistributedServer>(replica_radar_config,
                                                 "10.100.11.163", 9000),
      // machine 4 instance 7
      std::make_shared<ReplicaDistributedServer>(replica_radar_config,
                                                 "10.100.11.159", 8999),
      // machine 4 instance 8
      std::make_shared<ReplicaDistributedServer>(replica_radar_config,
                                                 "10.100.11.159", 9000), 
      // machine 5 instance 9
      std::make_shared<ReplicaDistributedServer>(replica_radar_config,
                                                 "10.100.11.153", 8999),
      // machine 5 instance 10
      std::make_shared<ReplicaDistributedServer>(replica_radar_config,
                                                 "10.100.11.153", 9000),
      // machine 6 instance 11
      std::make_shared<ReplicaDistributedServer>(replica_radar_config,
                                                 "10.100.11.138", 8999),
      // machine 6 instance 12
      std::make_shared<ReplicaDistributedServer>(replica_radar_config,
                                                 "10.100.11.138", 9000),
  };

  primaryServer->loadSensors();
  for (auto server : replicaServers) {
    server->loadSensors();

    for (auto& sensor : server->sensors) {
      if (sensor->config->type == "Lidar") {
        boost::asio::io_service io_service;
        boost::asio::ip::udp::socket socket(io_service);
        boost::asio::ip::udp::endpoint remote_endpoint;
        socket.open(boost::asio::ip::udp::v4());
        remote_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 2368);
        sensor->sampleCallback = [&remote_endpoint, &socket](DataFrame* frame){
            auto& lidarFrame = *static_cast<LidarFrame*>(frame);
            int count = 0;
            for(auto& packet : lidarFrame.packets){
                boost::system::error_code err;
                socket.send_to(boost::asio::buffer(&packet, sizeof(LidarPacket)), remote_endpoint, 0, err);
                std::this_thread::sleep_for(std::chrono::microseconds(1327));
            }
        };
      } else if (sensor->config->type == "Radar") {
        std::shared_ptr<ros::NodeHandle> node_handle_radar = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
        ros::Publisher pub_radar = node_handle_radar->advertise<monodrive_msgs::Radar>("/monodrive/radar", 1);
        sensor->sampleCallback = [pub_radar](DataFrame *frame) {
          auto& data = *static_cast<RadarFrame*>(frame);
          monodrive_msgs::Radar msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
          pub_radar.publish(msg);
        };
      }
    }
  }

  for (auto& sensor : primaryServer->sensors) {
    if (sensor->config->type == "State") {
      std::shared_ptr<ros::NodeHandle> node_handle_state = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
      ros::Publisher pub_state = node_handle_state->advertise<monodrive_msgs::StateSensor>("/monodrive/state", 1);
      sensor->sampleCallback = [pub_state](DataFrame *frame) {
        auto& data = *static_cast<StateFrame*>(frame);
        monodrive_msgs::StateSensor msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
        pub_state.publish(msg);
      };
    } 
    else if (sensor->config->type == "IMU") {
      std::shared_ptr<ros::NodeHandle> node_handle_imu = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
      ros::Publisher pub_imu = node_handle_imu->advertise<sensor_msgs::Imu>("/monodrive/imu", 1);
      sensor->sampleCallback = [pub_imu](DataFrame *frame) {
        auto& data = *static_cast<ImuFrame*>(frame);
        sensor_msgs::Imu msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
        pub_imu.publish(msg);
      };
    }
    else if (sensor->config->type == "Waypoint") {
      std::shared_ptr<ros::NodeHandle> node_handle_waypoint = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
      ros::Publisher pub_waypoint = node_handle_waypoint->advertise<monodrive_msgs::WaypointSensor>("/monodrive/waypoint", 1);
      sensor->sampleCallback = [pub_waypoint](DataFrame *frame) {
        auto& data = *static_cast<WaypointFrame*>(frame);
        monodrive_msgs::WaypointSensor msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
        pub_waypoint.publish(msg);
      };
    }
    else if (sensor->config->type == "GPS") {
      std::shared_ptr<ros::NodeHandle> node_handle_gps = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
      ros::Publisher pub_gps = node_handle_gps->advertise<sensor_msgs::NavSatFix>("/monodrive/gps", 1);
      sensor->sampleCallback = [pub_gps](DataFrame *frame) {
        auto& data = *static_cast<GPSFrame*>(frame);
        sensor_msgs::NavSatFix msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
        pub_gps.publish(msg);
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
  std::thread replicaThread(ReplicaThread, primaryServer, replicaServers);
  std::thread primaryThread(PrimaryThread, primaryServer);

  float fps = 100.f;
  rosLoop(fps);

  primaryThread.join();
  replicaThread.join();

  return 0;
}
