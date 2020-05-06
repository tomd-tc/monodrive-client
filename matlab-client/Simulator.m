classdef Simulator < handle
   properties 
      connection
      server_ip
      server_port
      ID_SIMULATOR_CONFIG = "SimulatorConfig_ID"
      ID_REPLAY_CONFIGURE_TRAJECTORY_COMMAND = "REPLAY_ConfigureTrajectoryCommand_ID"
      %ID_REPLAY_CONFIGURE_SENSORS_COMMAND = "REPLAY_ConfigureSensorsCommand_ID"
      ID_REPLAY_ReConfigureSensorCommand_ID  = "REPLAY_ReConfigureSensorCommand_ID"
      ID_REPLAY_STEP_SIMULATION_COMMAND = "REPLAY_StepSimulationCommand_ID"
      ID_REPLAY_STATE_SIMULATION_COMMAND = "REPLAY_StateStepSimulationCommand_ID"
      %ID_EGO_CONTROL = "EgoControl_ID"
      ID_SAMPLE_SENSORS = "SampleSensorsCommand_ID"
      ID_SPAWN_VEHICLE_COMMAND_ID = "SpawnVehicleCommand_ID"

      HEADER_CONTROL = uint32(hex2dec('6d6f6e6f'))
      HEADER_RESPONSE = uint32(hex2dec('6f6e6f6d'))
   end
   methods
       function initialize(obj, config)              
                obj.server_ip = config.server_ip;
                obj.server_port = config.server_port;
                obj.connection = tcpip(config.server_ip, config.server_port, 'Timeout', 2);
                obj.connection.OutputBufferSize = 10000000;
                obj.connection.ByteOrder = 'bigEndian';
                obj.connection.Terminator('');
                obj.connect()
       end
       function connect(obj)
            fopen(obj.connection);
       end

       function response = configure_simulator(obj, sim_config)
            command = obj.ID_SIMULATOR_CONFIG;
            response = obj.send_message(command, sim_config);
       end

       function response = configure_vehicle(obj)
            command = obj.ID_SPAWN_VEHICLE_COMMAND_ID;
            config = obj.vehicle_config;
            response = obj.send_message(command, config);
       end       

       function response = configure_scenario(obj, scenario)
            command = obj.ID_REPLAY_CONFIGURE_TRAJECTORY_COMMAND;
            config.frame = {scenario.frame}
            %response = obj.send_message(command, config);
             msg = struct(...
            'type', command,... 
            'message', {{config}},...
            'reference', randi(1000));
            disp(msg)
            data_length = uint32(length(jsonencode(msg)) + 8);
            fwrite(obj.connection,[obj.HEADER_CONTROL, data_length], 'uint32')
            msg_bytes = native2unicode(jsonencode(msg), 'UTF-8');
            fwrite(obj.connection, msg_bytes);

            response_header = fscanf(obj.connection, '%c', 4);
            response_length = fscanf(obj.connection, '%c', 4);
            response_length = double(response_length(4));
            response = fscanf(obj.connection, '%c', response_length-8)
       end
       
       function response = sample_sensors(obj)
            command = obj.ID_SAMPLE_SENSORS;
            %response = obj.send_message(command, "");
            
            msg = struct(...
            'type', command,... 
            'message', "",...
            'reference', randi(1000));
        
            data_length = uint32(length(jsonencode(msg)) + 8);
            fwrite(obj.connection,[obj.HEADER_CONTROL, data_length], 'uint32')
            msg_bytes = native2unicode(jsonencode(msg), 'UTF-8');
            fwrite(obj.connection, msg_bytes);

       end
       
%        function response = step_vehicle(obj, command, message)
%             msg = struct(...
%             'type', command,... 
%             'message', message,...
%             'reference', randi(1000));
% 
%             data_length = uint32(length(jsonencode(msg)) + 8);
%             fwrite(obj.connection,[obj.HEADER_CONTROL, data_length], 'uint32')
%             msg_bytes = native2unicode(jsonencode(msg), 'UTF-8');
%             fwrite(obj.connection, msg_bytes)
%        end
%            
       function response = send_message(obj, command, message)
            msg = struct(...
            'type', command,... 
            'message', message,...
            'reference', randi(1000));
        
            data_length = uint32(length(jsonencode(msg)) + 8);
            fwrite(obj.connection,[obj.HEADER_CONTROL, data_length], 'uint32')
            msg_bytes = native2unicode(jsonencode(msg), 'UTF-8');
            fwrite(obj.connection, msg_bytes);

            response_header = fscanf(obj.connection, '%c', 4);
            response_length = fscanf(obj.connection, '%c', 4);
            response_length = double(response_length(4));
            response = fscanf(obj.connection, '%c', response_length-8)
           end
        
   end
end