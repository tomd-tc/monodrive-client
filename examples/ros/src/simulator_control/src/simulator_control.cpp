#include <iostream>		  // std::cout
#include <chrono>		  // std::chrono	
#include <thread>         // std::thread
#include <memory>
#include <vector>

//monoDrive Includes
#include "Simulator.h"
#include "Configuration.h"  // holder for sensor, simulator, scenario, weather, and vehicle configurations
#include "Sensor.h"
#include "sensor_config.h"
#include "Stopwatch.h"
#include "MessageFactory.h"

#include <future>

#include "ros/ros.h"
#include "ros/package.h"

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

std::vector<std::shared_ptr<Sensor>> create_sensors_for(const Simulator& sim0, bool use_simulator_ros)
{
    std::vector<std::shared_ptr<Sensor>> sensors;

    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim0.getServerIp();
    vp_config.server_port = sim0.getServerPort();
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256,256);
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

    StateConfig state_config;
    state_config.server_ip = sim0.getServerIp();
    state_config.server_port = sim0.getServerPort();
    state_config.desired_tags = {"ego"};
    state_config.listen_port = 8101;
    state_config.debug_drawing = false;
    state_config.undesired_tags = {"static"};

    // here we configure sensors to publish directly to ros, or
    // to use the sensor callback to forward data to a ros topic
    if (use_simulator_ros) {
        state_config.ros.publish_to_ros = true;
        state_config.ros.advertise = true;
        state_config.ros.topic = "/monodrive/state_sensor";
        state_config.ros.message_type = "monodrive_msgs/StateSensor";
    } 
    // create the sensor
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<StateConfig>(state_config)));

    // if we're not using ros from the simulator, set up the callback based forwarding
    if (!use_simulator_ros) {
        std::cout << "setting up callback" << std::endl;
        std::shared_ptr<ros::NodeHandle> node_handle_state = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
        ros::Publisher pub_state = node_handle_state->advertise<monodrive_msgs::StateSensor>("/monodrive/state_sensor", 1);
        sensors.back()->sampleCallback = [pub_state](DataFrame *frame) {
            StateFrame data = *static_cast<StateFrame*>(frame);
            monodrive_msgs::StateSensor msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
            pub_state.publish(msg);
            std::cout << data.sample_count << std::endl;
        };
    }    

    WaypointConfig waypoint_config;
    waypoint_config.server_ip = sim0.getServerIp();
    waypoint_config.server_port = sim0.getServerPort();
    waypoint_config.distance = 1000.0;
    waypoint_config.frequency = 100.0;
    waypoint_config.listen_port = 8201;
    if (use_simulator_ros) {
        waypoint_config.ros.publish_to_ros = true;
        waypoint_config.ros.advertise = true;
        waypoint_config.ros.topic = "/monodrive/waypoint_sensor";
        waypoint_config.ros.message_type = "monodrive_msgs/WaypointSensor";
    }
    // create the sensor
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<WaypointConfig>(waypoint_config)));

    // if we're not using ros from the simulator, set up the callback based forwarding
    if (!use_simulator_ros) {
        std::shared_ptr<ros::NodeHandle> node_handle_waypoint = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
        ros::Publisher pub_waypoint = node_handle_waypoint->advertise<monodrive_msgs::WaypointSensor>("/monodrive/waypoint_sensor", 1);
        sensors.back()->sampleCallback = [pub_waypoint](DataFrame *frame) {
            WaypointFrame data = *static_cast<WaypointFrame*>(frame);
            monodrive_msgs::WaypointSensor msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
            pub_waypoint.publish(msg);
        };
    }

    IMUConfig imu_config;
    imu_config.server_ip = sim0.getServerIp();
    imu_config.server_port = sim0.getServerPort();
    imu_config.listen_port = 8102;
    if (use_simulator_ros) {
        imu_config.ros.publish_to_ros = true;
        imu_config.ros.advertise = true;
        imu_config.ros.topic = "/monodrive/imu_sensor";
        imu_config.ros.message_type = "sensor_msgs/Imu";
    }
    // create the sensor
    sensors.push_back(std::make_shared<Sensor>(std::make_unique<IMUConfig>(imu_config)));

    // if we're not using ros from the simulator, set up the callback based forwarding
    if (!use_simulator_ros) {
        std::shared_ptr<ros::NodeHandle> node_handle_imu = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
        ros::Publisher pub_imu = node_handle_imu->advertise<sensor_msgs::Imu>("/monodrive/imu_sensor", 1);
        sensors.back()->sampleCallback = [pub_imu](DataFrame *frame) {
            ImuFrame data = *static_cast<ImuFrame*>(frame);
            sensor_msgs::Imu msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data);
            pub_imu.publish(msg);
        };
    }

    std::cout<<"***********ALL SENSOR CONFIGS*******"<<std::endl;
    for (auto& sensor : sensors)
    {
        sensor->configure();
    }
    return sensors;
}

void run_monodrive(float fps, Simulator& sim, bool use_simulator_ros){
    std::cout << "Creating Sensors." << std::endl;
    std::vector<std::shared_ptr<Sensor>> sensors = create_sensors_for(sim, use_simulator_ros);
    std::cout << "Sensors configured!" << std::endl;

    ros::Rate rate(fps);

    while(ros::ok()){
        // Sample the sensors
        rate.sleep();

        if (!use_simulator_ros) {
            sim.sampleAll(sensors);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_follower");

    bool use_simulator_ros = true;
    ros::NodeHandle nh("~");
    nh.getParam("use_simulator_ros", use_simulator_ros);

    fs::path path(ros::package::getPath("simulator_control"));
    fs::path configPath = path / "config";

    //Read JSON files in cpp_client/config directory
    Configuration config(
        configPath / "simulator_infinity.json",
        configPath / "weather.json",
        configPath / "scenario_correct_map.json");
    
    if (!use_simulator_ros) {
        // remove the ros config if we're not publishing to ros directly from simulator
        config.simulator.erase("ros");
    }

    std::string server0_ip = config.simulator["server_ip"];
    int server_port = config.simulator["server_port"];
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()) {
        std::cerr << "Unable to configure simulator!" << std::endl;
        return -1;
    }

    /// initialize the vehicle
    float fps = 60.f;
    run_monodrive(fps, sim0, use_simulator_ros);

    return 0;
}
