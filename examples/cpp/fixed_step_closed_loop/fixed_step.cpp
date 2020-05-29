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

#include "LaneSpline.h"
#include "DataFrame.h"

using namespace lane_spline;

LaneSpline lanespline = LaneSpline(std::string("examples/cpp/lane_follower/Straightaway5k.json"));

#define IMG_WIDTH 1920
#define IMG_HEIGHT 1080

//Single Simulator Example
std::string server0_ip = "127.0.0.1";
int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;

std::vector<std::shared_ptr<Sensor>> create_sensors_for(const Simulator& sim0)
{
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

    StateConfig state_config;
    state_config.desired_tags = {"ego"};
    state_config.server_ip = sim0.getServerIp();
    state_config.server_port = sim0.getServerPort();
    state_config.listen_port = 8101;
    state_config.debug_drawing = true;
    state_config.undesired_tags = {""};
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<StateConfig>(state_config)));


    // Send configuraitons to the simulator
    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }
    return sensors;
}

EgoControlConfig planning(DataFrame* dataFrame){
    auto& frame = *static_cast<StateFrame*>(dataFrame);

    VehicleState* vehicle_frame = nullptr;
    for(auto& vehicle : frame.vehicles){
        for(auto& tag : vehicle.state.tags){
            if(tag == "ego"){
                vehicle_frame = &vehicle;
                break;
            }
        }
    }
    if(vehicle_frame == nullptr){
        std::cout << "No ego vehicle in frame." << std::endl;
        return EgoControlConfig();
    }

    Eigen::VectorXd position(3);
    position << vehicle_frame->state.odometry.pose.position.x,
        vehicle_frame->state.odometry.pose.position.y,
        vehicle_frame->state.odometry.pose.position.z;
    Eigen::Quaternion<double> orientation(
        vehicle_frame->state.odometry.pose.orientation.w,
        vehicle_frame->state.odometry.pose.orientation.x,
        vehicle_frame->state.odometry.pose.orientation.y,
        vehicle_frame->state.odometry.pose.orientation.z
    );
    auto nearestIndex = lanespline.GetNearestPoint("road_0", "lane_2", position);
    auto& lane_points = lanespline.spline_map["road_0"]["lane_2"];
    int nextPointIndex = nearestIndex;
    if(nearestIndex >= lane_points.size()-4){
        nextPointIndex = (int)lane_points.size() - 1;
    }
    else{
        nextPointIndex += 3;
    }
    Eigen::VectorXd forwardVector(3);
    forwardVector << 1, 0, 0;
    forwardVector = orientation * forwardVector;
    auto nextPoint = lane_points[nextPointIndex];
    Eigen::VectorXd dirToNextPoint = nextPoint - position;
    dirToNextPoint.normalize();

    double angle = -dirToNextPoint.head<3>().cross(forwardVector.head<3>())[2];

    EgoControlConfig egoControl;
    egoControl.forward_amount = 0.75;
    egoControl.brake_amount = 0.0;
    egoControl.drive_mode = 1;
    egoControl.right_amount = (float)angle;
    return egoControl;
}

void run_loop(Simulator& sim0){
    //Setup and Connect Sensors
    std::vector<std::shared_ptr<Sensor>> sensors = create_sensors_for(sim0);

    // start sensor sample loops so they are ready to receive data from the simulator
    for(auto& sensor : sensors){
        sensor->StartSampleLoop();
    }

    // Can register callbacks for sensors which trigger on the sensors thread
    // during sampling 
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
        // step the simulation one simulation frame
        ClosedLoopStepCommandConfig stepCommand;
        stepCommand.time_step = 0.01;
        sim0.send_command(ApiMessage(1234, ClosedLoopStepCommand_ID, true, stepCommand.dump()));
        // perception
        // tell simulator to send all sensor data frames to client sensors
        sim0.sample_all(sensors);
        // planning
        // besides callbacks can always access the latest data frame on the sensor
        EgoControlConfig egoControl = planning(sensors[1]->frame);
        // control
        // send ego control message calculated from our planning and control
        sim0.send_command(ApiMessage(777, EgoControl_ID, true, egoControl.dump()));
    }
}

int main(int argc, char** argv)
{
    //Read JSON files in cpp_client/config directory
    Configuration config(
        "examples/cpp/lane_follower/simulator_no_traffic.json",
        "config/weather.json",
        "config/scenario_config_multi_vehicle.json"
    );
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    // configure the simulator with scenario, weather, and environment information
    if(!sim0.configure()){
        return -1;
    }

    // start our main perception planning and control loop
    run_loop(sim0);
    
    return 0;
}
