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
namespace ds = distributed_server;

void run_monodrive(float fps)
{

  ros::Rate rate(fps);

  while (ros::ok())
  {
    // Sample the sensors
    rate.sleep();
  }
}

void PrimaryThread(std::shared_ptr<ds::DistributedServer> server)
{
  uint64_t control_time = 0;
  int frame_count = 0;
  mono::precise_stopwatch primary_stopwatch;

  std::cout << "Primary thread starting..." << std::endl;
  while (true)
  {
    auto control_start_time =
        primary_stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>();

    {
      server->Sample();
    }

    control_time +=
        primary_stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>() -
        control_start_time;

    // Spit out FPS about every second
    if ((++frame_count) % 100 == 0)
    {
      std::cout << "=== Control Avg: " << (control_time / frame_count) / 1e6
                << std::endl;
      std::cout << "FPS: " << frame_count / (control_time / 1e6) << std::endl;
      frame_count = 0;
      control_time = 0;
    }
  }
}

void ReplicaThread(
    std::vector<std::shared_ptr<ds::DistributedServer>> servers)
{
  uint64_t sample_time = 0;
  int frame_count = 0;
  mono::precise_stopwatch replica_stopwatch;
  std::string local_state_data("");

  std::cout << "Replica thread starting..." << std::endl;
  while (true)
  {
    // Cue the replicas and time them
    auto sample_start_time =
        replica_stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>();

    for (auto &server : servers)
    {
      server->Sample(true);
    }

    for (auto &server : servers)
    {
      while (server->IsSampling())
        ;
    }

    sample_time +=
        replica_stopwatch.elapsed_time<uint64_t, std::chrono::microseconds>() -
        sample_start_time;

    // Spit out FPS about every second
    if ((++frame_count) % 30 == 0)
    {
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

  // Set up all the server
  auto primary_server =
      std::make_shared<ds::DistributedServer>("127.0.0.1", 8999,
                                              ds::kServerType::PRIMARY);
  std::vector<std::shared_ptr<ds::DistributedServer>> replica_servers = {
      std::make_shared<ds::DistributedServer>("192.168.86.45", 8999,
                                              ds::kServerType::REPLICA),
      std::make_shared<ds::DistributedServer>("192.168.86.45", 9000,
                                              ds::kServerType::REPLICA)};

  // Configure the senors for each server
  if (!primary_server->Configure(
          {ds::kSensorType::VIEWPORT}) ||
      !replica_servers[0]->Configure(
          {ds::kSensorType::VIEWPORT, ds::kSensorType::RADAR}) ||
      !replica_servers[1]->Configure(
          {ds::kSensorType::VIEWPORT, ds::kSensorType::LIDAR}))
  {
    return -1;
  }

  std::shared_ptr<ros::NodeHandle> node_handle_state = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
  ros::Publisher pub_state = node_handle_state->advertise<monodrive_msgs::StateSensor>("/monodrive/state", 1);
  primary_server->AddSensor(ds::kSensorType::BINARY_STATE, [pub_state](DataFrame *frame) {
    StateFrame data = *static_cast<StateFrame*>(frame);
    monodrive_msgs::StateSensor msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
    pub_state.publish(msg);
  });

  std::shared_ptr<ros::NodeHandle> node_handle_imu = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
  ros::Publisher pub_imu = node_handle_state->advertise<sensor_msgs::Imu>("/monodrive/imu", 1);
  primary_server->AddSensor(ds::kSensorType::IMU, [pub_imu](DataFrame *frame) {
    ImuFrame data = *static_cast<ImuFrame*>(frame);
    sensor_msgs::Imu msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
    pub_imu.publish(msg);
  });


  std::shared_ptr<ros::NodeHandle> node_handle_waypoint = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
  ros::Publisher pub_waypoint = node_handle_state->advertise<monodrive_msgs::WaypointSensor>("/monodrive/state", 1);
  primary_server->AddSensor(ds::kSensorType::WAYPOINT, [pub_waypoint](DataFrame *frame) {
    WaypointFrame data = *static_cast<WaypointFrame*>(frame);
    monodrive_msgs::WaypointSensor msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
    pub_waypoint.publish(msg);
  });

  // Bench marking stuff
  std::thread primary_thread(PrimaryThread, primary_server);
  std::thread replica_thread(ReplicaThread, replica_servers);

  float fps = 100.f;
  run_monodrive(fps);

  primary_thread.join();
  replica_thread.join();

  return 0;
}
