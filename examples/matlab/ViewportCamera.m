classdef ViewportCamera < Sensor
    % Camera Sensor
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.
    
    % Public, tunable properties
    properties
        config_path = 'configurations/viewport_camera.json'
    end
    
    % Pre-computed constants
    properties(Access = protected)
        image = zeros(512,512,3);
        width
        height
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Setup super class then
            setupImpl@Sensor(obj);
            %obj.width = obj.config.stream_dimensions.x;
            %obj.height = obj.config.stream_dimensions.y;
        end
        
        function y = stepImpl(obj)
            y = [1];
        end
        
        function y = parse(obj, data)
            y = [];
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function c1 = isOutputFixedSizeImpl(obj)
            c1 = true;
        end
        function c1 = isOutputComplexImpl(obj)
            c1 = false;
        end
        function sizeout = getOutputSizeImpl(~)
            sizeout = 1;
        end
        
        function dataout = getOutputDataTypeImpl(~)
            dataout = 'double';
        end
    end
end