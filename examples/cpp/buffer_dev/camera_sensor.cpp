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

// #define IMG_WIDTH 4096
// #define IMG_HEIGHT 2160

#define IMG_WIDTH 1920
#define IMG_HEIGHT 1080

std::vector<std::shared_ptr<Sensor>> create_sensors_for(const Simulator& sim0)
{
    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;
    for(int i = 0; i < 5; ++i){
        CameraConfig fc_config;
        fc_config.server_ip = sim0.getServerIp();
        fc_config.server_port = sim0.getServerPort();
        fc_config.listen_port = 8100 + i;
        fc_config.location.z = 225;
        fc_config.rotation.pitch = -5;
        // Uncomment to receive grayscale images
        // fc_config.channels = "gray";
        fc_config.resolution = Resolution(IMG_WIDTH,IMG_HEIGHT);
        fc_config.annotation.include_annotation = true;
        fc_config.annotation.desired_tags = {"traffic_sign"};
        sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(fc_config)));
    }

    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim0.getServerIp();
    vp_config.server_port = sim0.getServerPort();
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256,256);
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    // Send configuraitons to the simulator
    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }
    return sensors;
}

void camera_test(Simulator& sim0){
    //Setup and Connect Sensors
    std::vector<std::shared_ptr<Sensor>> sensors = create_sensors_for(sim0);
    //Get number of steps in scenario and start timer
    mono::precise_stopwatch stopwatch;

    /// initialize the vehicle, the first control command spawns the vehicle
    nlohmann::json ego_msg;
    ego_msg["forward_amount"] =  0.0;
    ego_msg["right_amount"] = 0.0;
    ego_msg["brake_amount"] = 1.0;
    ego_msg["drive_mode"] = 1;
    sim0.send_command(ApiMessage(123, EgoControl_ID, true, ego_msg));

    for(auto& sensor : sensors){
        sensor->StartSampleLoop();
    }

    sensors[0]->sample_callback = [](DataFrame* frame) {
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
    };

    std::cout << "Sampling sensor loop" << std::endl;
    while(true)
    {	
        sim0.sample_all(sensors);
    }
}

int main(int argc, char** argv)
{
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;
    
    //Read JSON files in cpp_client/config directory
    Configuration config(
        "config/simulator_no_traffic.json",
        "config/weather.json",
        "config/scenario_config_single_vehicle.json"
    );
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    camera_test(sim0);
    
    return 0;
}
