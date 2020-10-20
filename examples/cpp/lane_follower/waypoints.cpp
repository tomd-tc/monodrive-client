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

#include "pid.h"


#define IMG_WIDTH 1280
#define IMG_HEIGHT 720

// test critera
#define DURATION 15  // seconds
#define TIMESTEP 0.01  // seconds
#define MAX_LANE_DEVIATION 20  // cm

// speed control
PID pid(0.0125f, 0.004f, 0.0025f, -1.0f, 1.0f);
float last_time = 0;
float desired_speed = 1000.0;

// simulator server
std::string server_ip = "127.0.0.1";
int server_port = 8999;

// results
float deviation = 0;


EgoControlConfig planning(DataFrame* stateFrame, DataFrame* waypointFrame) {
    StateFrame* state = static_cast<StateFrame*>(stateFrame);
    WaypointFrame* waypoints = static_cast<WaypointFrame*>(waypointFrame);

    // find ego
    VehicleState* ego = nullptr;
    for (auto& vehicle : state->vehicles){
        for(auto& tag : vehicle.state.tags){
            if(tag == "ego"){
                ego = &vehicle;
                break;
            }
        }
    }
    if (ego == nullptr) {
        std::cout << "No ego vehicle in frame." << std::endl;
        return EgoControlConfig();
    }

    // find lane waypoints
    Eigen::VectorXd curr(3);
    Eigen::VectorXd next(3);
    curr.setZero();
    for (const auto& actor : waypoints->actor_waypoints) {
        if (actor.actor_id == ego->state.name) {
            for(const auto& lane : actor.lanes) {
                if(lane.waypoints.size() >= 2) {
                    curr << lane.waypoints[0].location.x,
                        lane.waypoints[0].location.y,
                        lane.waypoints[0].location.z;
                    next << lane.waypoints[1].location.x,
                        lane.waypoints[1].location.y,
                        lane.waypoints[1].location.z;
                    break;
                }
            }
        }
    }
    if (curr.isZero()) {
        std::cout << "Unable to get ego lane waypoints."<< std::endl;
        return EgoControlConfig();
    }

    // use lane follower for steering
    Eigen::VectorXd position(3);
    position << ego->state.odometry.pose.position.x,
        ego->state.odometry.pose.position.y,
        ego->state.odometry.pose.position.z;
    Eigen::Quaternion<double> orientation(
        ego->state.odometry.pose.orientation.w,
        ego->state.odometry.pose.orientation.x,
        ego->state.odometry.pose.orientation.y,
        ego->state.odometry.pose.orientation.z
    );
    Eigen::VectorXd forwardVector(3);
    forwardVector << 1, 0, 0;
    forwardVector = orientation * forwardVector;
    Eigen::VectorXd dirToNextPoint = next - position;
    dirToNextPoint.normalize();

    double angle = -dirToNextPoint.head<3>().cross(forwardVector.head<3>())[2];

    // use PID speed controller for throttle
    Eigen::VectorXd velocity(3);
    velocity << ego->state.odometry.linear_velocity.x,
        ego->state.odometry.linear_velocity.y,
        ego->state.odometry.linear_velocity.z;
    double speed = velocity.norm();

    double dt = last_time ? state->game_time - last_time : 0.1;
    last_time = state->game_time;

    float throttle = pid.pid(desired_speed - speed, dt);

    // get lane deviation
    double dev = (curr - position).norm();
    if (dev > deviation) {
        deviation = dev;
    }

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

    // set to fixed timestep mode
    config.simulator["simulation_mode"] = 3;
    config.simulator["time_step"] = TIMESTEP;

    // setup simulator
    Simulator& sim = Simulator::getInstance(config, server_ip, server_port);

    if(!sim.configure()){
        return -1;
    }

    // configure sensors
    std::vector<std::shared_ptr<Sensor>> sensors;
    CameraConfig fc_config;
    fc_config.server_ip = sim.getServerIp();
    fc_config.server_port = sim.getServerPort();
    fc_config.listen_port = 8100;
    fc_config.location.z = 225;
    fc_config.rotation.pitch = -5;
    fc_config.resolution = Resolution(IMG_WIDTH, IMG_HEIGHT);
    fc_config.annotation.include_annotation = true;
    fc_config.annotation.desired_tags = {"traffic_sign"};
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(fc_config)));

    StateConfig state_config;
    state_config.desired_tags = {"ego", "vehicle"};
    state_config.undesired_tags = {"static"};
    state_config.server_ip = sim.getServerIp();
    state_config.server_port = sim.getServerPort();
    state_config.listen_port = 8101;
    state_config.debug_drawing = false;
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<StateConfig>(state_config)));

    WaypointConfig wp_config;
    wp_config.server_ip = sim.getServerIp();
    wp_config.server_port = sim.getServerPort();
    wp_config.listen_port = 8102;
    wp_config.distance = 10000;
    wp_config.frequency = 100;
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<WaypointConfig>(wp_config)));

    std::cout << "Configuring sensors..." << std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }

    // register perception callback
    sensors[0]->sampleCallback = perception;

    // execute test
    for (int i = 0; i < DURATION / TIMESTEP; ++i)
    {
        // step
        ClosedLoopStepCommandConfig stepCommand;
        stepCommand.time_step = TIMESTEP;
        sim.sendCommand(ApiMessage(1234, ClosedLoopStepCommand_ID, true, stepCommand.dump()));

        // sampling/perception
        sim.sampleAll(sensors);

        // planning
        EgoControlConfig egoControl = planning(sensors[1]->frame, sensors[2]->frame);

        // control
        sim.sendCommand(ApiMessage(777, EgoControl_ID, true, egoControl.dump()));

        if (deviation > MAX_LANE_DEVIATION) {
            std::cout << "Lane deviation exceeded limit: "
                << deviation <<" cm. Exiting early." << std::endl;
            break;
        }
    }

    // write test result
    ResultMetric metric;
    metric.name = "lane_deviation";
    metric.score = deviation;

    Result res;
    res.pass = deviation < MAX_LANE_DEVIATION;
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
