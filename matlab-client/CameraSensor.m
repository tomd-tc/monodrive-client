classdef CameraSensor < Sensor 
    % Camera Sensor
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
        config_path = '../configurations/camera.json'
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
            obj.width = obj.config.stream_dimensions.x;
            obj.height = obj.config.stream_dimensions.y;
        end

        function y = stepImpl(obj)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            response_header = fread(obj.sensor_channel, 4, 'uint8');
            response_length = fread(obj.sensor_channel, 4, 'uint8'); 
            response_length = fread(obj.sensor_channel, 4, 'uint8'); 
            response_length = 4*obj.width*obj.height;
            response = zeros(1,response_length);
            response = fread(obj.sensor_channel, response_length, 'uint8');
            if length(response) == response_length
                r = response(3:4:end);
                redChannel = reshape(r, [obj.width,obj.height])';
                g = response(2:4:end);
                greenChannel = reshape(g, [obj.width,obj.height])';
                b = response(1:4:end);
                blueChannel = reshape(b, [obj.width,obj.height])';
                obj.image = cat(3, redChannel, greenChannel, blueChannel)/255;
                imshow(obj.image);
            end
            y = obj.image;
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
            sizeout = [512 512 3];
        end
        
        function dataout = getOutputDataTypeImpl(~)
            dataout = 'double';
        end
    end
end