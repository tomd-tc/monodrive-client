#include <iostream>		  // std::cout
#include <chrono>		  // std::chrono	
#include <thread>         // std::thread
#include <memory>
#include <vector>
#include <future>

// monoDrive includes
#include "Simulator.h"
#include "Configuration.h"  // holder for sensor, simulator, scenario, weather, and vehicle configurations
#include "Sensor.h"
#include "sensor_config.h"
#include "DataFrame.h"
#include "Stopwatch.h"

#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "LaneSpline.h"
#include "pid.h"

using namespace lane_spline;

LaneSpline lanespline = LaneSpline(std::string("examples/cpp/lane_follower/Straightaway5k.json"));

#define IMG_WIDTH 1280
#define IMG_HEIGHT 720

// simulator server
std::string server0_ip = "127.0.0.1";
int server_port = 8999;

// speed control
PID pid(0.0125f, 0.004f, 0.0025f, -1.0f, 1.0f);
float last_time = 0;
float desired_speed = 1000.0;


EgoControlConfig planning(DataFrame* dataFrame) {
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

    // use lane follower for steering
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

    // use PID speed controller for throttle
    Eigen::VectorXd velocity(3);
    velocity << vehicle_frame->state.odometry.linear_velocity.x,
        vehicle_frame->state.odometry.linear_velocity.y,
        vehicle_frame->state.odometry.linear_velocity.z;
    double speed = velocity.norm();

    double dt = last_time > 0 ? frame.game_time - last_time : 0.1;
    last_time = frame.game_time;

    float throttle = pid.pid(desired_speed - speed, dt);

    // form controls response
    EgoControlConfig egoControl;
    egoControl.forward_amount = std::max(0.0f, throttle);
    egoControl.brake_amount = std::min(0.0f, throttle);
    egoControl.drive_mode = 1;
    egoControl.right_amount = (float)angle;
    return egoControl;
}

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
    // read JSON files from config directory
    Configuration config(
        "examples/config/simulator_straightaway.json",
        "examples/config/weather.json",
        "examples/config/scenario_multi_vehicle_straightaway.json"
    );
    // set to fixed timestep mode
    config.simulator["simulation_mode"] = 3;
    config.simulator["time_step"] = 0.01;

    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    // configure the simulator with scenario, weather, and environment information
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
    fc_config.resolution = Resolution(IMG_WIDTH,IMG_HEIGHT);
    fc_config.annotation.include_annotation = true;
    fc_config.annotation.desired_tags = {"traffic_sign"};
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(fc_config)));

    LidarConfig ld_config;
    ld_config.server_ip = sim0.getServerIp();
    ld_config.server_port = sim0.getServerPort();
    ld_config.listen_port = 8300;
    ld_config.location.z = 250;
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<LidarConfig>(ld_config)));

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

    // Send configurations to the simulator and start listening
    std::cout<<"***********ALL SENSOR's CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }

    // Can register callbacks for sensors which trigger on the sensors thread
    // during sampling
    sensors[0]->sampleCallback = perception;

    // define callback with forwarding to veloview
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service);
    boost::asio::ip::udp::endpoint remote_endpoint;
    socket.open(boost::asio::ip::udp::v4());
    remote_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 2368);
    sensors[1]->sampleCallback = [&remote_endpoint, &socket](DataFrame* frame){
        auto& lidarFrame = *static_cast<LidarFrame*>(frame);
        int count = 0;
        for(auto& packet : lidarFrame.packets){
            boost::system::error_code err;
            socket.send_to(boost::asio::buffer(&packet, sizeof(LidarPacket)), remote_endpoint, 0, err);
            // std::this_thread::sleep_for(std::chrono::microseconds(1327));
        }
    };

    // start our main perception planning and control loop
    std::cout << "Sampling sensor loop" << std::endl;
    while(true)
    {
        // step the simulation one simulation frame
        ClosedLoopStepCommandConfig stepCommand;
        stepCommand.time_step = 0.016;
        sim0.sendCommand(stepCommand.message());
        // perception
        // tell simulator to send all sensor data frames to client sensors
        sim0.sampleAll(sensors);
        // planning
        // besides callbacks can always access the latest data frame on the sensor
        EgoControlConfig egoControl = planning(sensors[2]->frame);
        // control
        // send ego control message calculated from our planning and control
        sim0.sendCommand(ApiMessage(777, EgoControl_ID, true, egoControl.dump()));
    }
    
    return 0;
}
