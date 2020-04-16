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

std::vector<std::shared_ptr<Sensor>> create_sensors_for(const std::string& ip)
{
    // Configure the sensors we wish to use
    std::vector<std::shared_ptr<Sensor>> sensors;
    CameraConfig fc_config;
    fc_config.server_ip = ip;
    fc_config.listen_port = 8100;
    fc_config.location.z = 225;
    fc_config.rotation.pitch = -5;
    fc_config.resolution = CameraConfig::Resolution(512,512);
    sensors.push_back(std::make_shared<Sensor>(fc_config));

    IMUConfig im_config;
    im_config.server_ip = ip;
    im_config.listen_port = 8101;
    sensors.push_back(std::make_shared<Sensor>(im_config));

    ViewportCameraConfig vp_config;
    vp_config.server_ip = ip;
    vp_config.location.z = 200;
    Sensor(vp_config).configure();

    GPSConfig gps_config;
    gps_config.server_ip = ip;
    gps_config.listen_port = 8102;
    sensors.push_back(std::make_shared<Sensor>(gps_config));

    RadarConfig radar_config;
    radar_config.location.x = 250;
    radar_config.location.z = 50;
    radar_config.server_ip = ip;
    radar_config.listen_port = 8103;
    sensors.push_back(std::make_shared<Sensor>(radar_config));

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

int main(int argc, char** argv)
{
    //Single Simulator Example
    std::string server0_ip = "127.0.0.1";
    int server_port = 8999;   // This has to be 8999 this simulator is listening for connections on this port;
    
    //Read JSON files in cpp_client/config directory
    Configuration config(
        "config/simulator.json",
        "config/vehicle.json",
        "config/weather.json",
        "config/scenario.json"
    );
    Simulator& sim0 = Simulator::getInstance(config, server0_ip, server_port);

    if(!sim0.configure()){
        return -1;
    }

    //Setup and Connect Sensors
    std::vector<std::shared_ptr<Sensor>> sensors = create_sensors_for(server0_ip);
    
    //Get number of steps in scenario and start timer
    int nSteps = config.scenario.size();
    int idx = 0;
    mono::precise_stopwatch stopwatch;

    //Step through scenario while reading sensor ouputs
    std::future<bool> stepTask;
    std::cout << "Running scenario" << std::endl;
    for(; idx < nSteps; idx++)
    {	
        std::vector<std::future<bool>> sampleTasks;
        //step simulator
        stepTask = std::async([&sim0, &idx](){
            return sim0.step(idx, 1);
        });
        //sample all sensors
        for(auto& sensor : sensors)
        {
            if(!sensor->sample())
                std::cout << "Failed to sample " << sensor->config->type << std::endl;
            sensor->parse();
        }
        // review senors, see replay example for better parallelization
        view_sensors(sensors);
        if(!stepTask.get()){
            break;
        }
    }
    //Calculate FPS
    auto scenario_time = stopwatch.elapsed_time<unsigned int, std::chrono::microseconds>();
    auto fps = float(idx+1)/scenario_time*1000000.0;
    std::cout<< "fps = " + std::to_string(fps) << std::endl;
    
    return 0;
}
