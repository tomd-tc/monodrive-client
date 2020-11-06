classdef Simulator < handle
    
    properties (Constant)
        ID_SIMULATOR_CONFIG = "SimulatorConfig_ID"
        ID_REPLAY_CONFIGURE_TRAJECTORY_COMMAND = "REPLAY_ConfigureTrajectoryCommand_ID"
        ID_REPLAY_CONFIGURE_SENSORS_COMMAND = "REPLAY_ConfigureSensorsCommand_ID"
        ID_REPLAY_ReConfigureSensorCommand_ID  = "REPLAY_ReConfigureSensorCommand_ID"
        ID_REPLAY_STEP_SIMULATION_COMMAND = "REPLAY_StepSimulationCommand_ID"
        ID_REPLAY_STATE_SIMULATION_COMMAND = "REPLAY_StateStepSimulationCommand_ID"
        ID_EGO_CONTROL = "EgoControl_ID"
        ID_SAMPLE_SENSORS = "SampleSensorsCommand_ID"
        ID_SPAWN_VEHICLE_COMMAND_ID = "SpawnVehicleCommand_ID"
        ID_CLOSED_LOOP_CONFIG_COMMAND = "ClosedLoopConfigCommand_ID"
        
        HEADER_CONTROL = uint32(hex2dec('6d6f6e6f'))
    end
    properties
        sim_config_json
        sim_config
        vehicle_config_json
        vehicle_config
        scenario_config_json
        scenario_config
        connection
        server_ip
        server_port
    end
    methods
        function initialize(obj)
            % TODO pass in programmatically
            
            sim_config_path = 'configurations/simulator.json';
            fid = fopen(sim_config_path,'r','n','UTF-8');
            obj.sim_config_json = fscanf(fid, '%s');
            obj.sim_config = struct(jsondecode(obj.sim_config_json));
            fclose(fid);
            
            vehicle_config_path = 'configurations/vehicle.json';
            fid = fopen(vehicle_config_path,'r','n','UTF-8');
            obj.vehicle_config_json = fscanf(fid, '%s');
            obj.vehicle_config = struct(jsondecode(obj.vehicle_config_json));
            fclose(fid);
            
            scenario_config_path = 'configurations/trajectories/scenario_config_single_vehicle.json';
            fid = fopen(scenario_config_path,'r','n','UTF-8');
            obj.scenario_config_json = fscanf(fid, '%s');
            obj.scenario_config = struct(jsondecode(obj.scenario_config_json));
            fclose(fid);
            
            obj.connect();
        end
        
        function configure(obj)
            % configure simulator
            obj.configure_simulator();
            obj.configure_scenario();
        end
        
        function connect(obj)
            % TODO check if connected
            % connect to simulator server
            obj.server_ip = obj.sim_config.server_ip;
            obj.server_port = obj.sim_config.server_port;
            obj.connection = tcpclient(obj.sim_config.server_ip, obj.sim_config.server_port, 'Timeout', 2);
            obj.connection.ByteOrder = 'big-endian';
            obj.connection.Terminator('');
        end
        
        function delete(obj)
            % cleanup here
        end
        
        function response = configure_simulator(obj)
            command = obj.ID_SIMULATOR_CONFIG;
            config = obj.sim_config;
            response = obj.send_message(command, config);
        end
        
        function response = configure_vehicle(obj)
            % do we need this?
            command = obj.ID_SPAWN_VEHICLE_COMMAND_ID;
            config = obj.vehicle_config;
            response = obj.send_message(command, config);
        end
        
        function response = configure_scenario(obj)
            command = obj.ID_CLOSED_LOOP_CONFIG_COMMAND;
            config.vehicles = {obj.scenario_config.vehicles};
            response = obj.send_message(command, config);
        end
        
        function response = sample_sensors(obj)
            command = obj.ID_SAMPLE_SENSORS;
            response = obj.send_message(command, "");
        end
        
        function response = step_vehicle(obj, forward_amount, right_amount, brake_amount)
            command = obj.ID_EGO_CONTROL;
            msg = ego_control_command(forward_amount, right_amount, brake_amount);
            response = obj.send_message(command, msg);
        end
        
        function response = send_message(obj, command, message)
            msg = struct(...
                'type', command,...
                'message', message,...
                'reference', randi(1000));
            
            % write command
            data_length = uint32(length(jsonencode(msg)) + 8);
            write(obj.connection,[obj.HEADER_CONTROL, data_length], 'uint32')
            msg_bytes = native2unicode(jsonencode(msg), 'UTF-8');
            write(obj.connection, msg_bytes);
            
            % read and parse response
            header = read(obj.connection, 2, 'uint32');
            magic = header(1);
            sz = header(2);
            if magic == obj.HEADER_CONTROL
                response = read(obj.connection, sz - 8, 'string');
            else
                disp("Unable to parse response.");
                response = "";
            end
        end
    end
end