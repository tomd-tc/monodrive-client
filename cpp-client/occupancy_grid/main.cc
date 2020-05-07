#include <iostream>
#include <thread>
#include <future>

#include "Simulator.h"

#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

std::vector<std::shared_ptr<Sensor>> create_sensors_for(const std::string& ip)
{
    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;
    OccupancyGridConfig occ_config;
    occ_config.server_ip = ip;
    occ_config.listen_port = 8100;
    occ_config.resolution = Resolution(512, 512);
    occ_config.meters_per_pixel = 0.1;
    sensors.push_back(std::make_shared<Sensor>(
      std::make_unique<OccupancyGridConfig>(occ_config)));

    ViewportCameraConfig vp_config;
    vp_config.server_ip = ip;
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256,256);
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    // Send configurations to the simulator
    std::cout << "***********ALL SENSOR's CONFIGS*******" << std::endl;
    for (auto& sensor : sensors) {
      std::cout << "Sensor:" << sensor->config->dump() << std::endl;
      sensor->configure();
    }
    return sensors;
}

int main(int argc, char** argv) {
  std::string server0_ip = "127.0.0.1";
  int server_port = 8999;  // This has to be 8999 this simulator is listening
                           // for connections on this port;
  
  // Read JSON files in cpp_client/config directory
  Configuration config("cpp-client/parser_dev/simulator.json",
                       "config/vehicle.json", "config/weather.json",
                       "cpp-client/buffer_dev/scenario.json");
  Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

  if (!sim0.configure()) {
    return -1;
  }

  // Setup and Connect Sensors
  std::vector<std::shared_ptr<Sensor>> sensors = create_sensors_for(server0_ip);

  /// initialize the vehicle, the first control command spawns the vehicle
  sim0.send_command(ApiMessage(123, EgoControl_ID, true,
                               {{"forward_amount", 0.0},
                                {"right_amount", 0.0},
                                {"brake_amount", 0.0},
                                {"drive_mode", 1}}));
  for (auto& sensor : sensors) {
    sensor->StartSampleLoop();
  }

  sensors[0]->sample_callback = [](DataFrame* frame) {
    auto camFrame = static_cast<CameraFrame*>(frame);
    auto imFrame = camFrame->imageFrame;
    cv::Mat img(imFrame->resolution.y, imFrame->resolution.x, CV_8UC1,
                imFrame->pixels);
    cv::imshow("monoDrive", img);
    cv::waitKey(1);
  };

  std::cout << "Sampling sensor loop" << std::endl;
  int count = 0;
  while (true) {
    sim0.sample_all(sensors);
  }

  return 0;
}