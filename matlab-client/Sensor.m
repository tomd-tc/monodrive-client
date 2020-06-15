classdef Sensor < matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime
    % Base Class for all Sensors

    % Public, tunable properties
    properties
        
    end

    properties(Nontunable)
        SampleTime = 1.4; % Sample Time
        OffsetTime = 0.2; % Offset Time
        TickTime = 0.1;
        SampleTimeTypeProp (1, 1) {mustBeMember(SampleTimeTypeProp, ...
            ["Discrete","FixedInMinorStep","Controllable",...
            "Inherited","InheritedNotControllable",...
            "InheritedErrorConstant"])} = "Discrete"
    end
    
    % Pre-computed constants
    properties(Access = protected)
        sim = libpointer
        sensor_channel = libpointer
        config_json = string
        config = struct()
    end
    
    properties(DiscreteState)
        Count
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

            obj.sim = Simulator();
            obj.sim.initialize();
            
            fid = fopen(obj.config_path,'r','n','UTF-8');
            obj.config_json = fscanf(fid, '%s');
            obj.config = struct(jsondecode(obj.config_json));
            response = obj.config_sensor()
            fclose(fid);
            
            if  ~strcmp(obj.config.type,"ViewportCamera")
                obj.config.listen_port
                obj.sensor_channel = tcpip(obj.sim.server_ip, obj.config.listen_port, 'Timeout', 2);
                fclose(obj.sensor_channel)
                obj.sensor_channel.OutputBufferSize = 10000;
                obj.sensor_channel.InputBufferSize = 1080*1080*4 + 8;
                obj.sensor_channel.ByteOrder = 'bigEndian';
                obj.sensor_channel.Terminator('');
                obj.connect_sensor();
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
       
       function connect_sensor(obj)
           fopen(obj.sensor_channel);
       end
        
        function response = config_sensor(obj)
            command = obj.sim.ID_REPLAY_CONFIGURE_SENSORS_COMMAND;
            response = jsondecode(obj.sim.send_message(command, obj.config));
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
    end
end


