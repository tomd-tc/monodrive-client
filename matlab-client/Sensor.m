classdef Sensor < matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime
    % Base Class for all Sensors

    % Public, tunable properties
    properties
      server_ip = "127.0.0.1"
      server_port = 8999
    end

    properties(Nontunable)
        ID_REPLAY_CONFIGURE_SENSORS_COMMAND = "REPLAY_ConfigureSensorsCommand_ID"
        HEADER_CONTROL = uint32(hex2dec('6d6f6e6f'))
        HEADER_RESPONSE = uint32(hex2dec('6f6e6f6d'))
        SampleTime = 1.0; % Sample Time
        OffsetTime = 0.2; % Offset Time
        TickTime = 0.1;
        SampleTimeTypeProp (1, 1) {mustBeMember(SampleTimeTypeProp, ...
            ["Discrete","FixedInMinorStep","Controllable",...
            "Inherited","InheritedNotControllable",...
            "InheritedErrorConstant"])} = "Discrete"
    end
    
    % Pre-computed constants
    properties(Access = protected)

        connection = libpointer
        sensor_channel = libpointer
        config_json = string
        config = struct()
    end
    
    properties(DiscreteState)
        Count
    end
    methods
%        function initialize(obj, sim_config)  
%                 obj.server_ip = sim_config.server_ip;
%                 obj.server_port = sim_config.server_port;
%                 obj.connection = tcpip(sim_config.server_ip, sim_config.server_port, 'Timeout', 2);
%                 obj.connection.OutputBufferSize = 10000000;
%                 obj.connection.ByteOrder = 'bigEndian';
%                 obj.connection.Terminator('');
%        end
    end
    
    methods(Access = protected)
        function sts = getSampleTimeImpl(obj)
            switch char(obj.SampleTimeTypeProp)
                case 'Inherited'
                    sts = createSampleTime(obj,'Type','Inherited');
                case 'InheritedNotControllable'
                    sts = createSampleTime(obj,'Type','Inherited',...
                        'AlternatePropagation','Controllable');
                case 'InheritedErrorConstant'
                    sts = createSampleTime(obj,'Type','Inherited',...
                        'ErrorOnPropagation','Constant');
                case 'FixedInMinorStep'
                    sts = createSampleTime(obj,'Type','Fixed In Minor Step');
                case 'Discrete'
                    sts = createSampleTime(obj,'Type','Discrete',...
                      'SampleTime',obj.SampleTime, ...
                      'OffsetTime',obj.OffsetTime);
                case 'Controllable'
                    sts = createSampleTime(obj,'Type','Controllable',...
                        'TickTime',obj.TickTime);
            end
        end
        

        
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
                        % Perform one-time calculations, such as computing constants
            obj.connection = tcpip(obj.server_ip, obj.server_port, 'Timeout', 2);
            obj.connection.OutputBufferSize = 10000000;
            obj.connection.ByteOrder = 'bigEndian';
            obj.connection.Terminator('');
            fopen(obj.connection);
            obj.config = json_read(obj.config_path);
            response = obj.config_sensor();
            
            if  ~strcmp(obj.config.type,"ViewportCamera")
                obj.config.listen_port
                obj.sensor_channel = tcpip(obj.server_ip, obj.config.listen_port, 'Timeout', 2);
                fclose(obj.sensor_channel)
                obj.sensor_channel.OutputBufferSize = 10000;
                obj.sensor_channel.InputBufferSize = 1080*1080*4 + 8;
                obj.sensor_channel.ByteOrder = 'bigEndian';
                obj.sensor_channel.Terminator('');
                fopen(obj.sensor_channel);
            end

            
        end
        
        function sizeout = getOutputSizeImpl(~)
            sizeout = [512 512 3];
        end
        
          function [sz,dt,cp] = getDiscreteStateSpecificationImpl(~,name)
             if strcmp(name,'Count')
                sz = [1 1];
                dt = 'double';
                cp = false;
             else
                error(['Error: Incorrect State Name: ', name.']);
             end
          end
          function dataout = getOutputDataTypeImpl(~)
             dataout = 'double';
          end

          function cplxout = isOutputComplexImpl(~)
             cplxout = false;
          end
          function fixedout = isOutputFixedSizeImpl(~)
             fixedout = true;
          end
          function flag = isInputSizeMutableImpl(~,idx)
             if idx == 1
               flag = true;
             else
               flag = false;
             end
          end

        
        function Count  = stepImpl(obj,u)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            Count = obj.Count + u;
            obj.Count = Count;
            %Time = getCurrentTime(obj);
            sts = getSampleTime(obj);
            if strcmp(sts.Type,'Controllable')
               setNumTicksUntilNextHit(obj,obj.Count);
            end
            %SampleTime = sts.SampleTime;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.Count = 0;
        end
        
        function response = config_sensor(obj)
            command = obj.ID_REPLAY_CONFIGURE_SENSORS_COMMAND;
            response = jsondecode(obj.send_message(command, obj.config));
        end
        
        function flag = isInactivePropertyImpl(obj,prop)
            flag = false;
            switch char(obj.SampleTimeTypeProp)
                case {'Inherited', ...
                        'InheritedNotControllable', ...
                        'FixedInMinorStep'}
                    if any(strcmp(prop,{'SampleTime','OffsetTime','TickTime'}))
                        flag = true;
                    end
                case 'Discrete'
                    if any(strcmp(prop,{'TickTime'}))
                        flag = true;
                    end
                case 'Controllable'
                    if any(strcmp(prop,{'SampleTime','OffsetTime'}))
                        flag = true;
                    end
            end
        end
        
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


