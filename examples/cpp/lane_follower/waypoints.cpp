// Lane keep and automatic cruise control system being tested
// against the monoDrive simulator in closed loop mode

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

// test critera
const unsigned int DURATION = 10000;  // ms
const float MAX_LANE_DEVIATION = 25;  // cm

// speed control
const float DESIRED_SPEED = 1000.0;
const float LOOK_AHEAD = 200;

PID pid(0.01f, 0.0005f, 0.0002f, -1.0f, 1.0f);
float last_time = 0;
float last_throttle = 0;

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

    double dt = last_time > 0 ? state->game_time - last_time : 0.1;
    last_time = state->game_time;

    float throttle = last_throttle;
    if (dt) {
        throttle = pid.pid(DESIRED_SPEED - speed, dt);
        last_throttle = throttle;
    }

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


int uut(int argc, char** argv, Job* job)
{
    // reset any jobs globals
    pid.reset();
    last_time = 0;
    last_throttle = 0;
    deviation = 0;

    // get configuration from job
    Configuration config = job->getConfig();

    // set to closed loop mode
    config.simulator["simulation_mode"] = 0;

    // setup simulator
    Simulator& sim = Simulator::getInstance(config, server_ip, server_port);

    if(!sim.configure()){
        return -1;
    }

    // configure sensors
    std::vector<std::shared_ptr<Sensor>> sensors;

    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim.getServerIp();
    vp_config.server_port = sim.getServerPort();
    vp_config.location.z = 400;
    vp_config.location.x = -800;
    vp_config.rotation.pitch = -15;
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    StateConfig state_config;
    state_config.desired_tags = {"ego", "vehicle"};
    state_config.undesired_tags = {"static"};
    state_config.server_ip = sim.getServerIp();
    state_config.server_port = sim.getServerPort();
    state_config.listen_port = 8101;
    state_config.debug_drawing = true;
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<StateConfig>(state_config)));

    WaypointConfig wp_config;
    wp_config.server_ip = sim.getServerIp();
    wp_config.server_port = sim.getServerPort();
    wp_config.listen_port = 8102;
    wp_config.distance = 2000;
    wp_config.frequency = LOOK_AHEAD;
    wp_config.draw_debug = true;
    wp_config.debug_tags = {"ego"};
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<WaypointConfig>(wp_config)));

    std::cout << "Configuring sensors..." << std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }

    // execute test
    mono::precise_stopwatch watch;
    int count = 0;
    while (watch.elapsed_time<unsigned int, std::chrono::milliseconds>() < DURATION)
    {
        // sampling/perception
        sim.sampleAll(sensors);

        // planning
        EgoControlConfig egoControl = planning(sensors[0]->frame, sensors[1]->frame);

        // control
        sim.sendCommand(ApiMessage(777, EgoControl_ID, true, egoControl.dump()));

        if (deviation > MAX_LANE_DEVIATION) {
            std::cout << "Lane deviation exceeded limit: "
                << deviation <<" cm. Exiting early." << std::endl;
            sim.sendControl(0.f, 0.f, 1.f, 1);
            break;
        }
        count++;
    }
    double fps = (double)count * 1000.0 / watch.elapsed_time<double, std::chrono::milliseconds>();

    // write test result
    ResultMetric metricDeviation;
    metricDeviation.name = "lane_deviation";
    metricDeviation.score = deviation;

    ResultMetric metricFPS;
    metricFPS.name = "fps";
    metricFPS.score = fps;

    Result res;
    res.pass = deviation < MAX_LANE_DEVIATION;
    res.message = "this job passed";
    res.metrics.push_back(metricDeviation);
    res.metrics.push_back(metricFPS);

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
