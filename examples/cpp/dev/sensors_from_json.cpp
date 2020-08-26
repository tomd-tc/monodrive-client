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

    // Load configuration from file
    Configuration config(
        "examples/config/simulator_straightaway.json",
        "examples/config/weather.json",
        "examples/config/scenario_multi_vehicle_straightaway.json",
        "examples/config/sensors.json"
    );
    Simulator& sim0 = Simulator::getInstance(config);

    // Load sensors from file
    std::vector<std::shared_ptr<Sensor>> sensors;
    config.loadSensors(sensors);

    // Configure simulator
    if(!sim0.configure()){
        return -1;
    }

    // Send configurations to the simulator
    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }

    /// initialize the vehicle, the first control command spawns the vehicle
    sim0.sendControl(0, 0, 1, 1);

    sensors[0]->sampleCallback = [](DataFrame *frame) {
        auto camFrame = static_cast<CameraFrame *>(frame);
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
    };

    std::cout << "Sampling sensor loop" << std::endl;
    while(true)
    {
        sim0.sampleAll(sensors);
    }

    return 0;
}
