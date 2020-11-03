#include <iostream>		  // std::cout
#include <chrono>		  // std::chrono	
#include <thread>         // std::thread
#include <memory>
#include <vector>
#include <future>
#include <thread>

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

#define IMG_WIDTH 1920
#define IMG_HEIGHT 1200


int main(int argc, char** argv)
{
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;

    //Read JSON files in cpp_client/config directory
    Configuration config(
        "examples/config/simulator_straightaway.json",
        "examples/config/weather.json",
        "examples/config/scenario_multi_vehicle_straightaway.json"
    );

    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;

    CameraConfig fc_config;
    fc_config.server_ip = sim0.getServerIp();
    fc_config.server_port = sim0.getServerPort();
    fc_config.listen_port = 8100;
    fc_config.location.z = 225;
    fc_config.rotation.pitch = -5;
    fc_config.resolution = Resolution(IMG_WIDTH, IMG_HEIGHT);
    fc_config.annotation.include_annotation = true;
    fc_config.annotation.desired_tags = {"traffic_sign"};
    sensors.push_back(
        std::make_shared<Sensor>(std::make_unique<CameraConfig>(fc_config)));

    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim0.getServerIp();
    vp_config.server_port = sim0.getServerPort();
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256,256);
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    // Send configurations to the simulator
    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }

    //Get number of steps in scenario and start timer
    mono::precise_stopwatch stopwatch;

    /// initialize the vehicle, the first control command spawns the vehicle
    sim0.sendControl(0, 0, 1, 1);

    sensors[0]->sampleCallback = [](DataFrame* frame){
        auto camFrame = static_cast<CameraFrame*>(frame);
        auto imFrame = camFrame->imageFrame;
        cv::Mat img(imFrame->resolution.y, imFrame->resolution.x, CV_8UC4, imFrame->pixels);
        for(auto& annotation : camFrame->annotationFrame->annotations){
            for(auto& bbox : annotation.second.bounding_boxes_2d)
            cv::rectangle(img, cv::Point(int(bbox.xmin), int(bbox.ymin)), cv::Point(int(bbox.xmax), int(bbox.ymax)), cv::Scalar(0,0,255));
        }
        cv::imshow("monoDrive", img);
        cv::waitKey(1);
    };

    std::cout << "Sampling sensor loop" << std::endl;
    while(true)
    {
      sim0.sendControl(0, 0, 1, 1);
      sim0.sampleAll(sensors);
    }

    return 0;
}
