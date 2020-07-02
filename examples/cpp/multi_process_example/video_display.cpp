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

#include "DataFrame.h"

#define IMG_WIDTH 1920
#define IMG_HEIGHT 1080

//Single Simulator Example
std::string server0_ip = "127.0.0.1";
int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;

void perception(DataFrame* dataFrame) {
    auto camFrame = static_cast<CameraFrame *>(dataFrame);
    auto imFrame = camFrame->imageFrame;
    cv::Mat img;
    if (imFrame->channels == 4)
    {
        img = cv::Mat(imFrame->resolution.y, imFrame->resolution.x, CV_8UC4,
                      imFrame->pixels);
    }
    else
    {
        img = cv::Mat(imFrame->resolution.y, imFrame->resolution.x, CV_8UC1,
                      imFrame->pixels);
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGRA);
    }
    for (auto &annotation : camFrame->annotationFrame->annotations)
    {
        for (auto &bbox : annotation.second.bounding_boxes_2d)
            cv::rectangle(img, cv::Point(int(bbox.xmin), int(bbox.ymin)),
                          cv::Point(int(bbox.xmax), int(bbox.ymax)),
                          cv::Scalar(0, 0, 255));
    }
    cv::imshow("monoDrive", img);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    Simulator& sim0 = Simulator::getInstance(server0_ip, server_port);
    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;
    CameraConfig fc_config;
    fc_config.server_ip = sim0.getServerIp();
    fc_config.server_port = sim0.getServerPort();
    fc_config.listen_port = 8100;
    fc_config.location.z = 225;
    fc_config.rotation.pitch = -5;
    fc_config.resolution = Resolution(IMG_WIDTH,IMG_HEIGHT);
    fc_config.annotation.include_annotation = true;
    fc_config.annotation.desired_tags = {"traffic_sign"};
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(fc_config)));

    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim0.getServerIp();
    vp_config.server_port = sim0.getServerPort();
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256,256);
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    // Send configurations to the simulator and start listening
    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }

    // Can register callbacks for sensors which trigger on the sensors thread
    // during sampling
    sensors[0]->sampleCallback = perception;

    // start our main perception planning and control loop
    std::cout << "Sampling sensor loop" << std::endl;
    while(true)
    {
        // perception
        sim0.sampleSensorList(sensors);
    }
    
    return 0;
}