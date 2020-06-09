#include <iostream>
#include <thread>
#include <future>

#include "Simulator.h"
#include "LaneSpline.h"

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

std::mutex data_snapshot_mutex;

struct DataSnapShot {
  CameraFrame* camera_frame;
  StateFrame* state_frame;
  lane_spline::LaneSpline* map_data;
} DATA_SNAPSHOT;

void CorrelateSnapshot() {
  std::lock_guard<std::mutex> lock(data_snapshot_mutex);

  // Get the map points within the certain radius of the vehicle
  auto ego_location = DATA_SNAPSHOT.state_frame->vehicles[0].state.odometry.pose.position;
  Eigen::VectorXd ego_vec(2);
  ego_vec << ego_location.x, ego_location.y;
  // TODO this function currently segfaults
  //auto pts = DATA_SNAPSHOT.map_data->GetPointsWithinRadius(ego_vec, 100.0);
  auto imFrame = DATA_SNAPSHOT.camera_frame->imageFrame;
  cv::Mat img(imFrame->resolution.y, imFrame->resolution.x, CV_8UC1,
              imFrame->pixels);
  cv::imshow("monoDrive", img);
  cv::waitKey(1);
}

void OccupancyGridCallback(DataFrame* frame) {
    auto camFrame = static_cast<CameraFrame*>(frame);
    std::lock_guard<std::mutex> lock(data_snapshot_mutex);
    DATA_SNAPSHOT.camera_frame = camFrame;
}

void StateSensorCallback(DataFrame* frame) {
  auto& state_frame = *static_cast<StateFrame*>(frame);
  std::lock_guard<std::mutex> lock(data_snapshot_mutex);
  DATA_SNAPSHOT.state_frame = &state_frame;
}

int main(int argc, char **argv)
{
  std::string server0_ip = "127.0.0.1";
  int server_port = 8999; // This has to be 8999 this simulator is listening
                          // for connections on this port;

  // Read JSON files in cpp_client/config directory
  Configuration config("examples/config/simulator_straightaway.json",
                       "examples/config/weather.json",
                       "examples/config/scenario_config_single_vehicle.json");
  Simulator &sim0 = Simulator::getInstance(config, server0_ip, server_port);

  if (!sim0.configure())
  {
    return -1;
  }

  // Configure the sensors we wish to use
  std::vector<std::shared_ptr<Sensor>> sensors;

  OccupancyGridConfig occ_config;
  occ_config.server_ip = sim0.getServerIp();
  occ_config.server_port = sim0.getServerPort();
  occ_config.listen_port = 8100;
  occ_config.resolution = Resolution(1920, 1080);
  occ_config.meters_per_pixel = 0.1;
  sensors.push_back(std::make_shared<Sensor>(
      std::make_unique<OccupancyGridConfig>(occ_config)));
  sensors.back()->sampleCallback = OccupancyGridCallback;

  StateConfig s_config;
  s_config.server_ip = sim0.getServerIp();
  s_config.server_port = sim0.getServerPort();
  s_config.listen_port = 8101;
  s_config.desired_tags = {"ego", "vehicle"};
  s_config.include_obb = true;
  s_config.debug_drawing = true;
  sensors.push_back(std::make_shared<Sensor>(
      std::make_unique<StateConfig>(s_config)));
  sensors.back()->sampleCallback = StateSensorCallback;

  // Configure the Viewport but don't add it to the sensors array
  ViewportCameraConfig vp_config;
  vp_config.server_ip = sim0.getServerIp();
  vp_config.server_port = sim0.getServerPort();
  vp_config.location.z = 200;
  vp_config.resolution = Resolution(256, 256);
  Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

  // Send configurations to the simulator
  std::cout << "***********ALL SENSOR's CONFIGS*******" << std::endl;
  for (auto &sensor : sensors)
  {
    sensor->configure();
  }

  /// initialize the vehicle, the first control command spawns the vehicle
  EgoControlConfig ego_control_config;
  ego_control_config.forward_amount = 0.5f;
  ego_control_config.right_amount = 0.0f;
  ego_control_config.brake_amount = 0.0f;
  ego_control_config.drive_mode = 1;
  sim0.sendCommand(ego_control_config.message());

  // Get the current map information
  MapConfig map_config;
  nlohmann::json map_response;
  sim0.sendCommand(map_config.message(), &map_response);

  // Fix the JSON string for the current map output
  // TODO: Update this when the simulator is updated
  std::string map_string = map_response.dump();
  map_string.erase(std::remove(map_string.begin(), map_string.end(), '\\'),
                   map_string.end());
  map_string = map_string.substr(1, map_string.size() - 2);
  map_response = nlohmann::json::parse(map_string);

  lane_spline::LaneSpline ls;

  DATA_SNAPSHOT.map_data = &ls;

  for (auto &sensor : sensors)
  {
    sensor->startSampleLoop();
  }

  std::cout << "Sampling sensor loop" << std::endl;
  while (true)
  {
    sim0.sampleAll(sensors);
    CorrelateSnapshot();
  }

  return 0;
}