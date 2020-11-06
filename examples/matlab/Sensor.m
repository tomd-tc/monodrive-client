classdef (Abstract) Sensor < matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime
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
        connection = libpointer
        config_json = string
        config = struct()
    end
    
    properties(DiscreteState)
        Count
    end
    
    methods (Abstract, Access = protected)
        parse(obj)
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
            fclose(fid);
            obj.configure();
            if  ~strcmp(obj.config.type, "ViewportCamera")
                obj.connect();
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
        
        
        function count = stepImpl(obj)
            % read header
            length = read(obj.connection, 1, 'uint32');
            wall_time = read(obj.connection, 1, 'uint32');
            game_time = read(obj.connection, 1, 'single');
            sample_count = read(obj.connection, 1, 'uint32');
            
            % read data body
            data = read(obj.connection, length - 16, 'uint8');
            
            % hit sensor parse function
            obj.parse(data);
            
            % update state
            obj.Count = obj.Count + 1;
            count = obj.Count;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.Count = 0;
        end
        
        function connect(obj)
            obj.connection = tcpclient(obj.sim.server_ip, obj.config.listen_port, 'Timeout', 2);
            obj.connection.ByteOrder = 'big-endian';
            obj.connection.Terminator('');
        end
        
        function response = configure(obj)
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


