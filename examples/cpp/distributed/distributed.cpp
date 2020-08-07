#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <chrono>

#include "Simulator.h"
#include "Configuration.h"
#include "Sensor.h"
#include "sensor_config.h"


nlohmann::json CURRENT_STATE_FRAME = nullptr;
bool NEW_STATE_DATA_AVAILABLE = false;

void stateCallback(DataFrame* frame) {
    auto& state_frame = *static_cast<StateFrame*>(frame);
    CURRENT_STATE_FRAME = {
        {"frame", 
            {
                {"vehicles", state_frame.vehicles}, 
                {"objects", state_frame.objects}
            }
        }
    };
    NEW_STATE_DATA_AVAILABLE = true;
};

typedef std::vector<std::shared_ptr<Sensor>> sensor_vec;

int main(int argc, char** argv) {
    // Read in JSON files for simulator configuration
    Configuration config(
        "examples/config/simulator.json",
        "examples/config/weather.json",
        "examples/config/scenario.json"
    );

    // Setup the list of simulators that will be used
    std::vector<std::pair<Simulator*, sensor_vec>> server_list = {
        std::make_pair(
            &Simulator::getInstance(config, "192.168.2.3", 8999), 
            sensor_vec()),
        std::make_pair(
            &Simulator::getInstance(config, "192.168.2.3", 9000), 
            sensor_vec())
    };

    // Setup all the sensors for each server in the list
    for(auto& server : server_list) {
        if(!server.first->configure()) {
            std::cerr << "ERROR! Unable to configure simulator: " 
                << server.first->getServerIp() << ":" 
                << server.first->getServerPort()
                << std::endl;
            return -1;
        }

        if(server.first->getServerPort() == 8999) {
            StateConfig s_config;
            s_config.server_ip = server.first->getServerIp();
            s_config.server_port = server.first->getServerPort();
            s_config.listen_port = 8101;
            s_config.desired_tags = {"vehicle", "dynamic"};
            s_config.include_obb = false;
            s_config.debug_drawing = true;
            server.second.emplace_back(std::make_shared<Sensor>(
                std::make_unique<StateConfig>(s_config)));
            server.second.back()->sampleCallback = stateCallback;
        }

        ViewportCameraConfig vp_config;
        vp_config.server_ip = server.first->getServerIp();
        vp_config.server_port = server.first->getServerPort();
        vp_config.location.z = 200;
        vp_config.resolution = Resolution(256,256);
        Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();

        for(auto& sensor : server.second) {
            if(!sensor->configure()) {
                std::cerr << "ERROR! Unable to configure sensor "
                          << sensor->config->type << " on port " 
                          << sensor->config->listen_port << " for server "
                          << server.first->getServerIp() << ":" 
                          << server.first->getServerPort() << std::endl;
                return -1;
            }
        }
    }

    std::cout << "Sampling sensor loop" << std::endl;
    for(int step = 0; step < config.scenario.size(); step++) {
        server_list[0].first->stepSampleAll(server_list[0].second, step, 1);
        // Spin while we're waiting on the new state data to come in
        while(!NEW_STATE_DATA_AVAILABLE) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(1));
        }
        NEW_STATE_DATA_AVAILABLE = false;
        std::cout << "Sending frame:" << CURRENT_STATE_FRAME << std::endl;
        for(size_t i = 1; i < server_list.size(); i++) {
            server_list[i].first->stateStepSampleAll(server_list[i].second, 
                CURRENT_STATE_FRAME);
        }
    }

    return 0;
}
