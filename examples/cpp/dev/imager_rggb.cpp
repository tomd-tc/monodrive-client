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

#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#define IMG_WIDTH 1920
#define IMG_HEIGHT 1080

int main(int argc, char** argv)
{
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;

    Configuration config(
        "examples/config/simulator_imager.json",
        "examples/config/weather.json",
        "examples/config/imager.json"
    );
    // swap to this for interesting scenario with a fisheye camera
    Simulator sim0(config, server0_ip, server_port);

    // Configure simulator
    if(!sim0.configure()){
        return -1;
    }

    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;

    CameraConfig fc_config;
    fc_config.resolution = Resolution(IMG_WIDTH,IMG_HEIGHT);
    fc_config.server_ip = sim0.getServerIp();
    fc_config.server_port = sim0.getServerPort();
    fc_config.listen_port = 8100;
    fc_config.location.x = 70;
    fc_config.location.z = 170;
    // fc_config.color_filter_array.cfa = "rccc";
    fc_config.color_filter_array.cfa = "rggb";
    fc_config.color_filter_array.use_cfa = true;

    // configure viewport settings for hdmi streaming, uncomment to stream to HDMI
    // fc_config.viewport.enable_viewport = true;
    // fc_config.viewport.fullscreen = true;
    // fc_config.viewport.window_size = Resolution(IMG_WIDTH,IMG_HEIGHT);
    // 0 is main monitor starting from the left
    // fc_config.viewport.monitor_number = 1;
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(fc_config)));

    CameraConfig rgb_config;
    rgb_config.server_ip = sim0.getServerIp();
    rgb_config.server_port = sim0.getServerPort();
    rgb_config.listen_port = 8101;
    rgb_config.location.x = 70;
    rgb_config.location.z = 170;
    rgb_config.resolution = Resolution(IMG_WIDTH,IMG_HEIGHT);
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(rgb_config)));

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

    int count1 = 0;
    sensors[0]->sampleCallback = [&count1](DataFrame *frame) {
        auto camFrame = static_cast<CameraFrame*>(frame);
        auto imFrame = camFrame->imageFrame;
        cv::Mat img;
        if (imFrame->channels == 4)
        {
            img = cv::Mat(imFrame->resolution.y, imFrame->resolution.x, CV_8UC4,
                          imFrame->pixels);
        }
        else
            return;
        
        // just to make the rgb image smaller when saved
        cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);
        // std::cout << (int)img.at<cv::Vec3b>(0,0)[0] << " " 
        //     << (int)img.at<cv::Vec3b>(0,0)[1] << " " 
        //     << (int)img.at<cv::Vec3b>(0,0)[2] << " " <<std::endl;
        cv::Mat1b rggb;
        cv::extractChannel(img, rggb, 2); 
        std::cout << rggb.size() << std::endl;

        cv::Mat color;
        cv::cvtColor(rggb, color, cv::COLOR_BayerBG2BGR);
        if(count1++ == 50){
            // saving a copy of one of the streamed images
            cv::imwrite("de_bayered.png", color);
            cv::imwrite("bayer.png", rggb);
        }
        cv::imshow("de-bayered", color);
        cv::imshow("bayer", rggb);
        cv::waitKey(1);
    };
    // an rgb rendered image for comparison
    int count2 = 0;
    sensors[1]->sampleCallback = [&count2](DataFrame *frame) {
        auto camFrame = static_cast<CameraFrame*>(frame);
        auto imFrame = camFrame->imageFrame;
        cv::Mat img;
        if (imFrame->channels == 4)
        {
            img = cv::Mat(imFrame->resolution.y, imFrame->resolution.x, CV_8UC4,
                          imFrame->pixels);
        }
        else
            return;
        if(count2++ == 50){
            cv::imwrite("rgb.png", img);
        }
        cv::imshow("rgb", img);
        cv::waitKey(1);
    };

    std::cout << "Sampling sensor loop" << std::endl;
    while(true)
    {
        sim0.sampleAll(sensors);
    }

    return 0;
}
