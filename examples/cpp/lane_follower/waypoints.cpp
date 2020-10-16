// 

#include <iostream>		  // std::cout
#include <chrono>		  // std::chrono	
#include <thread>         // std::thread
#include <memory>
#include <vector>
#include <future>

// monoDrive includes
#include "Simulator.h"
#include "Configuration.h"
#include "Sensor.h"
#include "sensor_config.h"
#include "DataFrame.h"
#include "Stopwatch.h"
#include "Jobs.h"

#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <Eigen/Eigen>


#define IMG_WIDTH 1280
#define IMG_HEIGHT 720

// passing critera
// TODO

// simulator server
std::string server_ip = "127.0.0.1";
int server_port = 8999;


EgoControlConfig planning(StateFrame* state, WaypointFrame* waypoints) {
    // TODO
    return EgoControlConfig();
}

void perception(DataFrame* dataFrame) {
    auto camFrame = static_cast<CameraFrame *>(dataFrame);
    auto imFrame = camFrame->imageFrame;
    cv::Mat img;
    if (imFrame->channels == 4)
    {
        img = cv::Mat(imFrame->resolution.y, imFrame->resolution.x, CV_8UC4, imFrame->pixels);
    }
    else
    {
        img = cv::Mat(imFrame->resolution.y, imFrame->resolution.x, CV_8UC1, imFrame->pixels);
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGRA);
    }
    for (auto &annotation : camFrame->annotationFrame->annotations)
    {
        for (auto &bbox : annotation.second.bounding_boxes_2d)
            cv::rectangle(
                img,
                cv::Point(int(bbox.xmin), int(bbox.ymin)),
                cv::Point(int(bbox.xmax), int(bbox.ymax)),
                cv::Scalar(0, 0, 255)
            );
    }
    cv::imshow("monoDrive", img);
    cv::waitKey(1);
}

int uut(int argc, char** argv, Job* job)
{
    // get configuration from job
    Configuration config = job->getConfig();

    // setup simulator
    Simulator& sim = Simulator::getInstance(config, server_ip, server_port);

    if(!sim.configure()){
        return -1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));

    // write test result
    ResultMetric metric;
    metric.name = "metric";
    metric.score = 0.0;

    Result res;
    res.pass = true;
    res.message = "this job passed";
    res.metrics.push_back(metric);

    job->setResult(res);

    // cleanup
    Simulator::clearInstances();

    return 0;
}

int main(int argc, char** argv)
{
    Job job(argc, argv);
    return job.run(uut);
}
