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

std::vector<std::shared_ptr<Sensor>> create_sensors_for(const std::string& ip)
{
    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;
    CameraConfig fc_config;
    fc_config.server_ip = ip;
    fc_config.listen_port = 8100;
    fc_config.location.z = 225;
    fc_config.rotation.pitch = -5;
    fc_config.resolution = Resolution(IMG_WIDTH,IMG_HEIGHT);
    fc_config.annotation.include_annotation = true;
    fc_config.annotation.desired_tags = {"traffic_sign"};
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(fc_config)));

    // IMUConfig im_config;
    // im_config.server_ip = ip;
    // im_config.listen_port = 8101;
    // sensors.push_back(std::make_shared<Sensor>(std::make_unique<IMUConfig>(im_config)));

    ViewportCameraConfig vp_config;
    vp_config.server_ip = ip;
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256,256);
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    // GPSConfig gps_config;
    // gps_config.server_ip = ip;
    // gps_config.listen_port = 8102;
    // sensors.push_back(std::make_shared<Sensor>(std::make_unique<GPSConfig>(gps_config)));

    // RadarConfig radar_config;
    // radar_config.location.x = 245;
    // radar_config.location.z = 60;
    // radar_config.server_ip = ip;
    // radar_config.listen_port = 8103;
    // sensors.push_back(std::make_shared<Sensor>(std::make_unique<RadarConfig>(radar_config)));

    // Send configuraitons to the simulator
    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }
    return sensors;
}

void view_sensors(std::vector<std::shared_ptr<Sensor>>& sensors){
    // cast the data frame of the sensor to the appropiate type
    auto camera = static_cast<ImageFrame*>(sensors[0]->frame);
    auto imu = static_cast<ImuFrame*>(sensors[1]->frame);
    auto gps = static_cast<GPSFrame*>(sensors[2]->frame);
    auto radar = static_cast<RadarTargetListFrame*>(sensors[3]->frame);
    // now access the data frame's data
    std::cout << "IMU    accel: " << imu->acc_x << ", " << imu->acc_y << ", " << imu->acc_z << std::endl;
    std::cout << "GPS:   lat,long: " << gps->lattitude << ", " << gps->longitude << " yaw: " << gps->yaw << std::endl;
    std::cout << "RADAR: num targets: " << radar->targets.size() << std::endl;
    if(radar->targets.size() > 0){
        auto& target = radar->targets[0];
        std::cout << "RADAR: target 1 ~ ";
        if(target.target_ids.size() > 0){
            std::cout << "id1: " << target.target_ids[0] << " ";         }
        std::cout << "range: " << target.range << " aoa: " << target.aoa << " velocity " << target.velocity << std::endl;
    }
}

ByteBuffer test(){
    AnnotationFrame2D fake;
    fake.name = "stuff";
    BoundingBox2D box;
    box.name = "hi";
    fake.bounding_boxes_2d.push_back(box);
    fake.bounding_boxes_2d.push_back(box);
    std::cout << nlohmann::json(fake).dump() << std::endl;
    DataFrame* front;
    CameraAnnotationFrame* annoteFrame = new CameraAnnotationFrame;
    front = annoteFrame;
    annoteFrame->annotations[fake.name] = fake;
    return front->write();
}

void test2(){
	CameraAnnotationFrame framey;
	ByteBuffer buffy = framey.write();
	auto jj = buffy.BufferToJson();
    std::cout << jj.dump() << std::endl;
    for(auto& x : jj){
        std::cout << x.dump() << std::endl;
    }
}

int main(int argc, char** argv)
{
    std::cout << "Hello World!" << std::endl;
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;
    
    //Read JSON files in cpp_client/config directory
    Configuration config(
        "cpp-client/parser_dev/simulator.json",
        "config/vehicle.json",
        "config/weather.json",
        "cpp-client/buffer_dev/scenario.json"
    );
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    //Setup and Connect Sensors
    std::vector<std::shared_ptr<Sensor>> sensors = create_sensors_for(server0_ip);
    
    //Get number of steps in scenario and start timer
    mono::precise_stopwatch stopwatch;

    //Step through scenario while reading sensor ouputs
    std::future<bool> stepTask;
    // std::cout << "Running scenario" << std::endl;
    /// initialize the vehicle, the first control command spawns the vehicle
    sim0.send_command(ApiMessage(123, EgoControl_ID, true, 
        {   {"forward_amount", 0.0}, 
            {"right_amount", 0.0},
            {"brake_amount", 0.0},
            {"drive_mode", 1}
        }));
    for(auto& sensor : sensors){
        sensor->sample();
    }
    bool bContinue = true;

    sensors[0]->sample_callback = [](DataFrame* frame){
        auto camFrame = static_cast<CameraFrame*>(frame);
        // std::cout << "callback " << camFrame->annotationFrame->annotations.size() << std::endl;
        auto imFrame = camFrame->imageFrame;
        cv::Mat img(imFrame->resolution.y, imFrame->resolution.x, CV_8UC4, imFrame->pixels);
        for(auto& annotation : camFrame->annotationFrame->annotations){
            // std::cout << annotation.first << ": " << std::endl << nlohmann::json(annotation.second).dump(1) << std::endl;
            for(auto& bbox : annotation.second.bounding_boxes_2d)
            cv::rectangle(img, cv::Point(int(bbox.xmin), int(bbox.ymin)), cv::Point(int(bbox.xmax), int(bbox.ymax)), cv::Scalar(0,0,255));
        }
        cv::imshow("win", img);
        cv::waitKey(1);
    };

    std::thread t1([&sensors, &bContinue](){
        while(bContinue){
            // mono::precise_stopwatch watch;
            // ImageFrame* frame = static_cast<ImageFrame*>(sensors[0]->frame);
            // CameraFrame* frame = static_cast<CameraFrame*>(sensors[0]->frame);
            // std::cout << (int)frame->data[int(IMG_HEIGHT*IMG_WIDTH*0.5)] << " " << (int)frame->data[int(IMG_HEIGHT*IMG_WIDTH*0.5)] << " " << (int)frame->data[int(IMG_HEIGHT*IMG_WIDTH*0.5)] << std::endl;
            // cv::Mat img(frame->imageFrame->resolution.y, frame->imageFrame->resolution.x, CV_8UC4, frame->imageFrame->pixels);
            // for(auto& annotation : frame->annotationFrame->annotations){
            //     std::cout << annotation.first << std::endl;
            // }
            // cv::imshow("win", img);
            // cv::waitKey(1);
            // std::cout << watch.elapsed_time<unsigned int, std::chrono::milliseconds>() << " (ms)" << std::endl;
        }
    });
    std::cout << "Sampling sensor loop" << std::endl;
    int count = 0;
    while(true)
    {	
        // std::cout << "Sampling Sensors" << std::endl;
        ApiMessage sampleMessage(999, SampleSensorsCommand_ID, true, {});
        for(auto& sensor : sensors){
            sensor->sampleInProgress.store(true, std::memory_order::memory_order_relaxed);
        }
        sim0.send_command(sampleMessage);
        bool samplingInProgress = true;
        do{
            samplingInProgress = false;
            for(auto& sensor : sensors){
                if(sensor->sampleInProgress.load(std::memory_order::memory_order_relaxed)){
                    samplingInProgress = true;
                    break;
                }
            }
        } while(samplingInProgress);
    }
    bContinue = false;
    // t1.join();
    //Calculate FPS
    
    return 0;
}
