classdef (Abstract) Sensor < matlab.System & matlab.system.mixin.Propagates
    % Base Class for all Sensors
    
    % Public, tunable properties
    properties
    end
    
    % Public, non-tunable properties
    properties(Nontunable)
    end
    
    % Abstract
    properties(Abstract)
        config_path
    end
    
    % Pre-computed constants
    properties(Access = protected)
        sim = libpointer
        connection = libpointer
        config_json = string
        config = struct()
    end
    
    properties(DiscreteState)
        count
    end
    
    methods (Abstract, Access = protected)
        parse(obj)
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            obj.sim = Simulator();
            obj.sim.initialize();
            
            fid = fopen(obj.config_path,'r','n','UTF-8');
            obj.config_json = fscanf(fid, '%s');
            obj.config = struct(jsondecode(obj.config_json));
            fclose(fid);
            obj.configure();
            if ~strcmp(obj.config.type, "ViewportCamera")
                obj.connect();
            end
        end
        
        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(~,name)
            if strcmp(name,'count')
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
        
        function y = stepImpl(obj)
            % read header
            length = read(obj.connection, 1, 'uint32');
            wall_time = read(obj.connection, 1, 'uint32');
            game_time = read(obj.connection, 1, 'single');
            sample_count = read(obj.connection, 1, 'uint32');
            
            % read data body
            data = read(obj.connection, length - 16, 'uint8');
            
            % hit sensor parse function
            y = obj.parse(data);
            
            % update count
            obj.count = obj.count + 1;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.count = 0;
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
    end
end
