#include <iostream>		  // std::cout
#include <chrono>		  // std::chrono	
#include <thread>         // std::thread
#include <memory>
#include <vector>
#include <future>

//monoDrive Includes
#include "Simulator.h"
#include "Configuration.h"  // holder for sensor, simulator, scenario, weather, and vehicle configurations
#include "Sensor.h"
#include "sensor_config.h"
#include "Stopwatch.h"

#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


int main(int argc, char** argv)
{
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;

    //Read JSON files in cpp_client/config directory
    Configuration config(
        "examples/config/simulator.json",
        "examples/config/weather.json",
        "examples/config/trajectories/Car-to-Car-Cut-In.json"
    );
    config.simulator["map"] = "Straightaway5k";
    config.simulator["simulation_mode"] = 2;
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    // Setup and Connect Sensors
    std::vector<std::shared_ptr<Sensor>> sensors;
    CameraConfig fc_config;
    fc_config.server_ip = server0_ip;
    fc_config.listen_port = 8103;
    fc_config.location.z = 200;
    fc_config.resolution = Resolution(1920,1080);
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(fc_config)));

    IMUConfig imu_config;
    imu_config.server_ip = server0_ip;
    imu_config.listen_port = 8105;
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<IMUConfig>(imu_config)));

    ViewportCameraConfig vp_config;
    vp_config.location.z = 400;
    vp_config.location.x = -800;
    vp_config.rotation.pitch = -15;
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }

    // define callback for camera
    sensors[0]->sampleCallback = [](DataFrame* frame) {
      auto camFrame = static_cast<CameraFrame*>(frame);
      auto imFrame = camFrame->imageFrame;
      cv::Mat img;
      if(imFrame->channels == 4) {
        img = cv::Mat(imFrame->resolution.y, imFrame->resolution.x, CV_8UC4,
                      imFrame->pixels);
      } else {
        img = cv::Mat(imFrame->resolution.y, imFrame->resolution.x, CV_8UC1,
                      imFrame->pixels);
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGRA);
      }
      for (auto& annotation : camFrame->annotationFrame->annotations) {
        for (auto& bbox : annotation.second.bounding_boxes_2d)
          cv::rectangle(img, cv::Point(int(bbox.xmin), int(bbox.ymin)),
                        cv::Point(int(bbox.xmax), int(bbox.ymax)),
                        cv::Scalar(0, 0, 255));
      }
      cv::imshow("monoDrive", img);
      cv::waitKey(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    };

    //Get number of steps in scenario and start timer
    int nSteps = (int)config.scenario.size();
    int idx = 0;

    //Step through scenario while reading sensor ouputs
    std::cout << "Running scenario" << std::endl;
    for(; idx < nSteps; idx++)
    {	
        sim0.stepSampleAll(sensors, idx, 1);
    }
    
    return 0;
}
